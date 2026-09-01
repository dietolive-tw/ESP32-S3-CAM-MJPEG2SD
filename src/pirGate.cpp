/*
  PIR-gated camera power management.

  When pirGate is enabled, the camera is kept in standby (deinitialized) until
  a PIR rising edge (or an incoming web stream / still capture request) wakes
  it. After waking, FOMO confirmation runs for up to pirGateArmSecs; if no
  target is detected in that window the camera returns to standby. Once a
  recording starts, normal motion/record logic takes over; when the recording
  ends the camera stays active for pirGatePostRecSecs before going back to
  standby. After boot, the camera stays active for pirGateBootIdleSecs to allow
  the user to access the web UI without a PIR trigger.

  s60sc 2026
*/

#include "appGlobals.h"

#if INCLUDE_PERIPH

// ---------- user configuration ----------
bool pirGate = false;                 // enable PIR-gated camera power management
uint16_t pirGateArmSecs = 20;          // FOMO confirmation window after PIR trigger
uint16_t pirGateIdleSecs = 30;        // (reserved) idle time after PIR-only false alarm
uint16_t pirGatePostRecSecs = 30;     // idle time after recording ends before standby
uint16_t pirGateBootIdleSecs = 60;    // idle time after boot before first standby

// ---------- internal state ----------
typedef enum { CAM_ACTIVE, CAM_STANDBY } CamState;
static volatile CamState camState = CAM_ACTIVE;
static volatile uint32_t lastPIRms = 0;       // last PIR rising edge (ms)
static volatile uint32_t lastRecEndMs = 0;    // last recording end (ms)
static volatile uint32_t bootMs = 0;          // boot completion time (ms)
static volatile int32_t streamClients = 0;    // number of active web/stream/RTSP consumers
static volatile bool recInProgress = false;    // recording in progress flag
static volatile bool camReady = false;        // camera initialized and ready for use
static TaskHandle_t pirGateTaskHandle = NULL;
static SemaphoreHandle_t pirWakeSem = NULL;   // signaled by PIR ISR
static SemaphoreHandle_t camMutex = NULL;     // serializes suspend/resume
static portMUX_TYPE streamMux = portMUX_INITIALIZER_UNLOCKED;

// Forward declarations
static void suspendCam();
static bool resumeCam();
static void pirGateTask(void*);

// ---------- public API ----------

void pirGateSetup() {
  if (!pirGate) return;
  static bool initialized = false;
  if (initialized) return;
  initialized = true;
  pirWakeSem = xSemaphoreCreateBinary();
  camMutex = xSemaphoreCreateMutex();
  bootMs = millis();
  lastPIRms = 0;
  lastRecEndMs = 0;
  streamClients = 0;
  recInProgress = false;
  camReady = true;        // camera is up at boot
  camState = CAM_ACTIVE;
  xTaskCreate(pirGateTask, "pirGateTask", 4096, NULL, 1, &pirGateTaskHandle);
  LOG_INF("PIR gating enabled: arm=%us postRec=%us bootIdle=%us",
          pirGateArmSecs, pirGatePostRecSecs, pirGateBootIdleSecs);
}

// Called by PIR ISR (rising edge): signal pirGateTask to wake camera if needed.
void IRAM_ATTR pirGatePIRisr() {
  lastPIRms = millis();
  if (pirWakeSem != NULL) xSemaphoreGiveFromISR(pirWakeSem, NULL);
}

// Called by web/stream/still handlers before any camera use.
// Blocks until camera is ready (max ~5s). Returns true on success.
// Also increments streamClients to prevent suspension during use.
bool pirGateRequireCam() {
  if (!pirGate) return true;
  // Signal pirGateTask to resume camera if in standby
  if (camState != CAM_ACTIVE || !camReady) {
    if (pirWakeSem != NULL) xSemaphoreGive(pirWakeSem);
    // Wait for camReady (poll, since resume is async)
    uint32_t deadline = millis() + 5000;
    while (millis() < deadline) {
      if (camState == CAM_ACTIVE && camReady) break;
      delay(50);
    }
    if (!(camState == CAM_ACTIVE && camReady)) {
      LOG_WRN("pirGateRequireCam: timeout waiting for camera");
      return false;
    }
  }
  portENTER_CRITICAL(&streamMux);
  streamClients++;
  portEXIT_CRITICAL(&streamMux);
  return true;
}

// Called by web/stream/still handlers after camera use ends.
// Decrements streamClients; pirGateTask will suspend after timeout once all clients leave.
void pirGateReleaseCam() {
  if (!pirGate) return;
  portENTER_CRITICAL(&streamMux);
  if (streamClients > 0) streamClients--;
  portEXIT_CRITICAL(&streamMux);
}

// Called by processFrame() when a recording ends.
void pirGateNotifyRecEnd() {
  if (!pirGate) return;
  lastRecEndMs = millis();
}

// Called by processFrame() to indicate recording in progress (for suspend suppression).
void pirGateSetRecInProgress(bool inProgress) {
  if (!pirGate) return;
  recInProgress = inProgress;
}

// ---------- internal helpers ----------

static void suspendCam() {
  if (camState == CAM_STANDBY) return;
  xSemaphoreTake(camMutex, portMAX_DELAY);
  // Re-check under mutex: PIR may have triggered a recording since the decision.
  if (recInProgress || streamClients > 0) {
    xSemaphoreGive(camMutex);
    return;
  }
  LOG_INF("PIR gate: suspending camera");
  // Stop the frame timer so captureTask blocks on ulTaskNotifyTake.
  // This stops all camera frame processing without the risk of
  // esp_camera_deinit() racing with captureTask.
  controlFrameTimer(false);
  camReady = false;
  camState = CAM_STANDBY;
  xSemaphoreGive(camMutex);
  debugMemory("pirGate:suspend");
}

static bool resumeCam() {
  if (camState == CAM_ACTIVE && camReady) return true;
  xSemaphoreTake(camMutex, portMAX_DELAY);
  if (camState == CAM_ACTIVE && camReady) {
    xSemaphoreGive(camMutex);
    return true;
  }
  LOG_INF("PIR gate: resuming camera");
  // Camera hardware is still initialized; just restart the frame timer.
  setFPS(FPS);
  camReady = true;
  camState = CAM_ACTIVE;
  // Refresh lastPIRms so the arm timeout gives pirGateArmSecs before
  // the next suspend decision (prevents immediate re-suspend).
  lastPIRms = millis();
  xSemaphoreGive(camMutex);
  debugMemory("pirGate:resume");
  return true;
}

static void pirGateTask(void*) {
  // 1 second tick + PIR semaphore
  uint32_t lastDebug = 0;
  while (true) {
    // Wait up to 1s for PIR event; if none, run timeout check.
    if (xSemaphoreTake(pirWakeSem, pdMS_TO_TICKS(1000)) == pdTRUE) {
      // PIR fired (or requireCam signaled). Resume camera if in standby.
      if (camState == CAM_STANDBY) resumeCam();
    }
    if (!pirGate) continue;

    uint32_t now = millis();

    // Periodic debug every 30 seconds (only when in STANDBY, to confirm task alive)
    if (now - lastDebug > 30000 && camState == CAM_STANDBY) {
      lastDebug = now;
      LOG_INF("pirGate: STANDBY, lastPIRms=%lu lastRecEndMs=%lu",
              lastPIRms, lastRecEndMs);
    }

    // NOTE: Do NOT poll the PIR pin level here. The RCWL-0516 / typical PIR
    // sensors have a multi-second hold time (pin stays HIGH after triggering).
    // Polling would keep refreshing lastPIRms and prevent standby.
    // lastPIRms is updated only by the rising-edge ISR (pirGatePIRisr).

    if (camState == CAM_ACTIVE) {
      // Don't suspend while recording or while web/stream clients are active.
      if (recInProgress || streamClients > 0) continue;

      uint32_t lastActivity = bootMs;
      uint32_t timeout = pirGateBootIdleSecs * 1000UL;
      if (lastRecEndMs > lastActivity) {
        lastActivity = lastRecEndMs;
        timeout = pirGatePostRecSecs * 1000UL;
      }
      if (lastPIRms > lastActivity) {
        lastActivity = lastPIRms;
        timeout = pirGateArmSecs * 1000UL;
      }
      if (now - lastActivity > timeout) suspendCam();
    }
  }
}

#endif  // INCLUDE_PERIPH
