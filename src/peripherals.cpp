
// Optional peripherals, to support:
// - pin sensors eg PIR / radar
// - servos, eg camera pan / tilt / steer
// - DS18B20 temperature sensor
// - battery voltage measurement
// - lamp LED driver (PWM or WS2812 / SK6812)
// - 3 pin joystick 
// - MY9221 based LED Bar, eg 10 segment Grove LED Bar
// - 5 wire 28BYJ-48 Unipolar Stepper Motor with ULN2003 Motor Driver
// - 4 wire Bipolar Stepper Motor with MX1508 H-Bridge Motor Driver
//
// Peripherals can be hosted directly on the client ESP, or on
// a separate IO Extender ESP if the client ESP has limited free 
// pins, eg ESP-Cam module
// External peripherals should have low data rate and not require fast response,
// so interrupt driven input pins should be monitored internally by the client.
// Peripherals that need a clocked data stream such as microphones are not suitable
//
// Pin numbers must be > 0.
//
// s60sc 2022 - 2025
//

#include "appGlobals.h"

#if INCLUDE_PERIPH
#include "driver/ledc.h"

// peripherals used
bool pirUse; // true to use PIR for motion detection
bool ledBarUse; // true to use led bar
uint8_t lampLevel; // brightness of on board lamp led 
bool lampAuto = false; // if true in conjunction with pirUse and accelUse, switch on lamp when PIR or accelerometer activated at night
bool lampNight; // if true, lamp comes on at night (not used)
int lampType; // how lamp is used
bool voltUse; // true to report on ADC pin eg for for battery
bool stickUse; // true to use joystick
bool buzzerUse; // true to use buzzer
bool stepperUse; // true to use stepper motor
bool SVactive; // true to use servos
TaskHandle_t heartBeatHandle = NULL;
bool RCactive = false;

// Pins used by peripherals

// sensors 
int pirPin; // if pirUse is true
int lampPin;
int buzzerPin; // if buzzerUse is true

// Camera servos 
int servoPanPin;
int servoTiltPin;

// ambient / module temperature reading 
int ds18b20Pin; // if INCLUDE_DS18B20 true

// batt monitoring 
// only pin 33 can be used on ESP32-Cam module as it is the only available analog pin
int voltPin; 

// additional peripheral configuration
// configure for specific servo model, eg for SG90
int servoMinAngle; // degrees
int servoMaxAngle;
int servoMinPulseWidth; // usecs
int servoMaxPulseWidth;
int servoDelay; // control rate of change of servo angle using delay
int servoCenter = 90; // angle in degrees where servo is centered 

// configure battery monitor
int voltDivider; // set battVoltageDivider value to be divisor of input voltage from resistor divider
                 // eg: 100k / 100k would be divisor value 2
float voltLow; // voltage level at which to send out email alert
int voltInterval; // interval in minutes to check battery voltage

// buzzer duration
int buzzerDuration; // time buzzer sounds in seconds 

// RC pins and control
int servoSteerPin;
int lightsRCpin;
int heartbeatRC;
int maxSteerAngle;
int maxDutyCycle;
int minDutyCycle;
int maxTurnSpeed;
bool allowReverse;
bool autoControl;
int waitTime; 
int stickzPushPin; // digital pin connected to switch output
int stickXpin; // analog pin connected to X output
int stickYpin; // analog pin connected to Y output
int relayPin;
bool relayMode;

// MY9221 LED Bar pins
int ledBarClock;
int ledBarData;

// Stepper motor driver pins
#define stepperPins 4
#define stepperCount 2 // two independent ULN2003 stepper motors (0 = pan, 1 = tilt)
uint8_t stepINpins[stepperCount][stepperPins];

#if defined(PCF8575_ADDR)
#include <Wire.h>
static bool usePCF8575 = false;
static uint8_t pcf8575Port[2] = {0x00, 0x00}; // shadow registers for port 0 and port 1
static TwoWire pcfWire = TwoWire(1);
static SemaphoreHandle_t pcfMutex = NULL;

// Auxiliary lights on PCF8575 (manual web control only)
int auxLightPin[3] = {4, 5, 6}; // PCF8575 pin numbers (0-15) for white, red, IR
bool auxLightOn[3] = {false, false, false};

void pcf8575Init() {
  pcfWire.begin(PCF8575_SDA, PCF8575_SCL, 400000);
  pcfWire.setTimeOut(50);
  if (pcfMutex == NULL) pcfMutex = xSemaphoreCreateMutex();
  pcfWire.beginTransmission(PCF8575_ADDR);
  pcfWire.write((uint8_t)0x00); // port 0 all low
  pcfWire.write((uint8_t)0x00); // port 1 all low
  pcfWire.endTransmission();
  usePCF8575 = true;
  LOG_INF("PCF8575 initialised at 0x%02X on SDA:%d SCL:%d", PCF8575_ADDR, PCF8575_SDA, PCF8575_SCL);
}

static void pcf8575WritePort(uint8_t portIdx, uint8_t val, uint8_t mask = 0xFF) {
  // mask=0xFF (default) overwrites whole byte; mask=0x0F only modifies lower nibble
  if (pcfMutex != NULL && xSemaphoreTake(pcfMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    LOG_WRN("pcf8575WritePort mutex timeout");
    return;
  }
  pcf8575Port[portIdx] = (pcf8575Port[portIdx] & ~mask) | (val & mask);
  pcfWire.beginTransmission(PCF8575_ADDR);
  // PCF8575: first byte = port 0, second byte = port 1
  pcfWire.write(pcf8575Port[0]);
  pcfWire.write(pcf8575Port[1]);
  pcfWire.endTransmission();
  if (pcfMutex != NULL) xSemaphoreGive(pcfMutex);
}

static void pcf8575WritePin(uint8_t pcfPin, bool val) {
  // bit-level write to PCF8575 pin (0-15) preserving other bits via shadow register
  if (pcfPin > 15) return;
  uint8_t portIdx = pcfPin / 8;
  uint8_t bitPos = pcfPin % 8;
  uint8_t mask = 1 << bitPos;
  pcf8575WritePort(portIdx, val ? mask : 0, mask);
}

void setAuxLight(uint8_t lightIdx, bool on) {
  // control one of 3 auxiliary lights (0=white, 1=red, 2=IR) via PCF8575
  if (lightIdx >= 3 || !usePCF8575) return;
  uint8_t pcfPin = auxLightPin[lightIdx];
  if (pcfPin > 15) return;
  auxLightOn[lightIdx] = on;
  pcf8575WritePin(pcfPin, on);
  LOG_INF("Aux light %u (PCF8575 P%u) %s", lightIdx, pcfPin, on ? "ON" : "OFF");
}
#endif

static void doStep(uint8_t stepperIdx);
void setStickTimer(bool restartTimer, uint32_t interval);
void setLamp(uint8_t lampVal);


// individual pin sensor / controller functions

bool getPIRval() {
  // get PIR or radar sensor status 
  return digitalRead(pirPin); 
}

void buzzerAlert(bool buzzerOn) {
  // control active buzzer operation
  if (buzzerUse) {
    if (buzzerOn) {
      // turn buzzer on
      pinMode(buzzerPin, OUTPUT);
      digitalWrite(buzzerPin, HIGH); 
    } else digitalWrite(buzzerPin, LOW); // turn buzzer off
  }
}

// Control a Pan-Tilt-Camera stand using two servos connected to pins specified above
// Or control an RC servo
// Only tested for SG90 style servos
// Typically, wiring is:
// - orange: signal
// - red: 5V
// - brown: GND
//
#define PWM_FREQ 50 // hertz
#define DUTY_BIT_DEPTH 12 // max for ESP32-C3 is 14

TaskHandle_t servoHandle = NULL;
static int newTiltVal, newPanVal, newSteerVal;
static int oldPanVal, oldTiltVal, oldSteerVal; 

static int dutyCycle (int angle) {
  // calculate duty cycle for given angle
  angle = constrain(angle, servoMinAngle, servoMaxAngle);
  int pulseWidth = map(angle, servoMinAngle, servoMaxAngle, servoMinPulseWidth, servoMaxPulseWidth);
  return pow(2, DUTY_BIT_DEPTH) * pulseWidth * PWM_FREQ / USECS;
}

static int changeAngle(uint8_t servoPin, int newVal, int oldVal, bool useDelay = true) {
  // change angle of given servo
  if (newVal != oldVal) {
    int incr = newVal - oldVal > 0 ? 1 : -1;
    for (int angle = oldVal; angle != newVal + incr; angle += incr) {
      ledcWrite(servoPin, dutyCycle(angle));
      if (useDelay) delay(servoDelay); // set rate of change
    }
  }
  return newVal;
}

static void servoTask(void* pvParameters) {
  // update servo position from user input
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (newSteerVal != oldSteerVal) oldSteerVal = changeAngle(servoSteerPin, newSteerVal, oldSteerVal, false);
    if (newPanVal != oldPanVal) oldPanVal = changeAngle(servoPanPin, newPanVal, oldPanVal);
    if (newTiltVal != oldTiltVal) oldTiltVal = changeAngle(servoTiltPin, newTiltVal, oldTiltVal);
  }
}

void setCamPan(int panVal) {
  // change camera pan angle
  if (stepperUse && stepINpins[0][0] > 0) {
    // stepper pan: map 0~180 angle to 0~100% position
    int pct = constrain(panVal, 0, 180) * 100 / 180;
    int32_t targetSteps;
    if (panCalibrated) {
      int32_t travel = panCalSteps[1] - panCalSteps[0];
      targetSteps = panCalSteps[0] + (int32_t)(travel * (int32_t)pct) / 100;
    } else {
      int32_t cur = stepperGetPosition(0);
      targetSteps = stepperGetSteps(0) + ((int32_t)pct - cur) * (int32_t)stepperStepsPerRev(BYJ_48) / 100;
    }
    int32_t diff = targetSteps - stepperGetSteps(0);
    if (diff != 0) {
      bool cw = diff > 0;
      int steps = abs(diff);
      if (steps > 0) stepperRun(0, 10.0, (float)steps / stepperStepsPerRev(BYJ_48), cw, BYJ_48);
    }
    stepperSetPosition(0, (uint8_t)pct);
  } else if (servoPanPin && servoHandle != NULL) {
    newPanVal = panVal;
    xTaskNotifyGive(servoHandle);
  }
}

void setCamTilt(int tiltVal) {
  // change camera tilt angle
  if (stepperUse && stepINpins[1][0] > 0) {
    // stepper tilt: map 0~180 angle to 0~100% position
    int pct = constrain(tiltVal, 0, 180) * 100 / 180;
    int32_t targetSteps;
    if (tiltCalibrated) {
      int32_t travel = tiltCalSteps[1] - tiltCalSteps[0];
      targetSteps = tiltCalSteps[0] + (int32_t)(travel * (int32_t)pct) / 100;
    } else {
      int32_t cur = stepperGetPosition(1);
      targetSteps = stepperGetSteps(1) + ((int32_t)pct - cur) * (int32_t)stepperStepsPerRev(BYJ_48) / 100;
    }
    int32_t diff = targetSteps - stepperGetSteps(1);
    if (diff != 0) {
      bool cw = diff > 0;
      int steps = abs(diff);
      if (steps > 0) stepperRun(1, 10.0, (float)steps / stepperStepsPerRev(BYJ_48), cw, BYJ_48);
    }
    stepperSetPosition(1, (uint8_t)pct);
  } else if (servoTiltPin && servoHandle != NULL) {
    newTiltVal = tiltVal;
    xTaskNotifyGive(servoHandle);
  }
}

void trackMotionObject() {
  // auto-track motion object using stepper pan/tilt
  // called every frame from processFrame() when trackMotion is enabled
  // motionCentroidX/Y: 0.0~1.0 (0.5 = center), -1.0 = no target
  //
  // motionCentroid reflects changed pixels (background subtraction);
  // when there is no change the function simply returns to avoid
  // drifting on a static scene.
  if (!trackMotion || !stepperUse) return;
  // yield to manual pan/tilt control from web UI for 5 seconds after last manual command
  if (millis() < manualStepperUntilMs) return;

  if (motionCentroidX < 0.0 || motionCentroidY < 0.0) return; // no target: just wait

  // dead zone: don't move if object is near center (avoid jitter)
  const float deadZone = 0.15; // ±15% from center = no movement
  float offsetX = motionCentroidX - 0.5; // negative = left of center, positive = right
  float offsetY = motionCentroidY - 0.5; // negative = above center, positive = below
  // swap axes if camera sensor is rotated 90° vs real-world orientation
  if (trackSwap) {
    float tmp = offsetX;
    offsetX = offsetY;
    offsetY = tmp;
  }
  // invert direction to match camera mirror / mechanical mounting
  offsetX = -offsetX;
  offsetY = -offsetY;
  
  // independently decide whether pan/tilt need adjustment (one axis should not block the other)
  bool wantPan = fabsf(offsetX) >= deadZone && stepINpins[0][0] > 0 && !stepperIsRunning(0);
  bool wantTilt = fabsf(offsetY) >= deadZone && stepINpins[1][0] > 0 && !stepperIsRunning(1);
  // throttle stepper commands to once per second to match the ~1 Hz motion
  // check rate; without this, trackMotionObject would issue commands every
  // frame (~50ms) using the same stale centroid, causing the stepper to run
  // continuously
  static uint32_t lastTrackStepMs = 0;
  const uint32_t trackIntervalMs = 1000;
  bool throttle = (millis() - lastTrackStepMs) < trackIntervalMs;
  if (throttle) { wantPan = false; wantTilt = false; }
  if (wantPan || wantTilt) LOG_INF("track: cx=%.2f cy=%.2f ox=%.2f oy=%.2f swap=%d pan=%d tilt=%d run0=%d run1=%d",
    motionCentroidX, motionCentroidY, offsetX, offsetY, (int)trackSwap, wantPan, wantTilt,
    stepperIsRunning(0), stepperIsRunning(1));
  if (!wantPan && !wantTilt) return;
  
  // map offset to angle adjustment: larger offset = more movement
  // At 10 RPM, 28BYJ-48 takes ~6 sec per rev, so keep adjustments small
  const float trackSensitivity = 10.0; // max degrees per frame adjustment
  
  if (wantPan) {
    int curAngle = (int)stepperGetPosition(0) * 180 / 100;
    int adjustment = (int)(offsetX * trackSensitivity * 2.0);
    int newAngle = constrain(curAngle + adjustment, 0, 180);
    if (newAngle != curAngle) setCamPan(newAngle);
  }
  
  if (wantTilt) {
    int curAngle = (int)stepperGetPosition(1) * 180 / 100;
    int adjustment = (int)(offsetY * trackSensitivity * 2.0);
    int newAngle = constrain(curAngle + adjustment, 0, 180);
    if (newAngle != curAngle) setCamTilt(newAngle);
  }
  lastTrackStepMs = millis();
}

void setSteering(int steerVal) {
  // change steering angle
  newSteerVal = steerVal;
  if (servoSteerPin && servoHandle != NULL) xTaskNotifyGive(servoHandle);
}

static void prepServos() {
  // When stepper motors are active, skip servo pan/tilt setup to avoid
  // ledcAttach claiming pins that the stepper driver uses via digitalWrite
  bool stepperPanActive = stepperUse && stepINpins[0][0] > 0;
  bool stepperTiltActive = stepperUse && stepINpins[1][0] > 0;
  if (SVactive) {
    if (servoPanPin && !stepperPanActive) {
      ledcAttach(servoPanPin, PWM_FREQ, DUTY_BIT_DEPTH);
      ledcWrite(servoPanPin, dutyCycle(servoCenter)); // write center directly
    } else if (stepperPanActive) LOG_INF("Servo pan skipped - stepper active");
    else LOG_WRN("No servo pan pin defined");
    if (servoTiltPin && !stepperTiltActive) {
      ledcAttach(servoTiltPin, PWM_FREQ, DUTY_BIT_DEPTH);
      ledcWrite(servoTiltPin, dutyCycle(servoCenter));
    } else if (stepperTiltActive) LOG_INF("Servo tilt skipped - stepper active");
    else LOG_WRN("No servo tilt pin defined");
  }
  if (RCactive && servoSteerPin) {
    ledcAttach(servoSteerPin, PWM_FREQ, DUTY_BIT_DEPTH);
    ledcWrite(servoSteerPin, dutyCycle(servoCenter));
  }
  // initialise old values to match the physical position just written
  oldPanVal = oldTiltVal = oldSteerVal = servoCenter;

  // Only create servo task if there are real servo pins to manage (not just stepper dummies)
  bool needServoTask = (SVactive && !stepperPanActive && !stepperTiltActive && (servoPanPin || servoTiltPin))
                    || (RCactive && servoSteerPin);
  if (needServoTask) {
    xTaskCreateWithCaps(&servoTask, "servoTask", SERVO_STACK_SIZE, NULL, SERVO_PRI, &servoHandle, STACK_MEM);
    newPanVal = newTiltVal = newSteerVal = servoCenter;
    LOG_INF("Servo pin usage: %d, %d, %d", servoPanPin, servoTiltPin, servoSteerPin);
  }
}


/* Read temperature from DS18B20 connected to pin specified above
    Use Arduino Manage Libraries to install OneWire and DallasTemperature
    DS18B20 is a one wire digital temperature sensor
    Pin layout from flat front L-R: Gnd, data, 3V3.
    Need a 4.7k resistor between 3V3 and data line
    Runs in its own task as there is a 750ms delay to get temperature

    If DS18B20 is not present, use ESP internal temperature sensor
*/

#if INCLUDE_DS18B20
#if __has_include("../libraries/DallasTemperature/DallasTemperature.h") 
#include <OneWire.h> // https://github.com/PaulStoffregen/OneWire
#include <DallasTemperature.h> // https://github.com/milesburton/Arduino-Temperature-Control-Library
#else
#error "Need to install DallasTemperature and OneWire libraries"
#endif
#endif

// configuration
static float dsTemp = NULL_TEMP;
TaskHandle_t DS18B20handle = NULL;
static bool haveDS18B20 = false;

static void DS18B20task(void* pvParameters) {
#if INCLUDE_DS18B20
  // get current temperature from DS18B20 device
  OneWire oneWire(ds18b20Pin);
  DallasTemperature sensors(&oneWire);
  while (true) {
    dsTemp = NULL_TEMP;
    sensors.begin();
    uint8_t deviceAddress[8];
    sensors.getAddress(deviceAddress, 0);
    if (deviceAddress[0] == 0x28) {
      uint8_t tryCnt = 10;
      while (tryCnt) {
        sensors.requestTemperatures(); 
        dsTemp = sensors.getTempCByIndex(0);
        // ignore occasional duff readings
        if (dsTemp > NULL_TEMP) tryCnt = 10;
        else tryCnt--;
        delay(1000);
      }   
    } 
    // retry setting up ds18b20
    delay(10000);
  }
#endif
}

void prepTemperature() {
#if INCLUDE_DS18B20
  if (ds18b20Pin) {
    xTaskCreateWithCaps(&DS18B20task, "DS18B20task", DS18B20_STACK_SIZE, NULL, DS18B20_PRI, &DS18B20handle, STACK_MEM); 
    haveDS18B20 = true;
    LOG_INF("Using DS18B20 sensor");
  } else LOG_WRN("No DS18B20 pin defined, using chip sensor if present");
#endif
}

float readTemperature(bool isCelsius, bool onlyDS18) {
  // return latest read temperature value in celsius (true) or fahrenheit (false), unless error
  if (onlyDS18) return dsTemp;
  if (!haveDS18B20) dsTemp = readInternalTemp();
  return (dsTemp > NULL_TEMP) ? (isCelsius ? dsTemp : (dsTemp * 1.8) + 32.0) : dsTemp;
}

float getNTCcelsius (uint16_t resistance, float oldTemp) {
  // convert NTC thermistor resistance reading to celsius
  double Temp = log(resistance);
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp);
  Temp = (Temp == 0) ? oldTemp : Temp - 273.15; // if 0 then didnt get a reading
  return (float) Temp;
}

/************ battery monitoring ************/

// Read voltage from battery connected to ADC pin
// input battery voltage may need to be reduced by voltage divider resistors to keep it below 3V3.
static float currentVoltage = -1.0; // no monitoring
TaskHandle_t battHandle = NULL;

float readVoltage()  {
  return currentVoltage;
}

static void battTask(void* parameter) {
  if (voltInterval < 1) voltInterval = 1;
  while (true) {
    // convert analog reading to corrected voltage.  analogReadMilliVolts() not working
    currentVoltage = (float)(smoothAnalog(voltPin)) * 3.3 * voltDivider / MAX_ADC;

    static bool sentExtAlert = false;
    if (currentVoltage < voltLow && !sentExtAlert) {
      sentExtAlert = true; // only sent once per esp32 session
      char battMsg[20];
      sprintf(battMsg, "Voltage is %0.2fV", currentVoltage);
      externalAlert("Low battery", battMsg);
    }
    delay(voltInterval * 60 * 1000); // mins
  }
  vTaskDelete(NULL);
}

static void setupBatt() {
  if (voltUse) {
  	if (voltPin) {
      xTaskCreateWithCaps(&battTask, "battTask", BATT_STACK_SIZE, NULL, BATT_PRI, &battHandle, STACK_MEM);
      LOG_INF("Monitor batt voltage");
      debugMemory("setupBatt");
    } else LOG_WRN("No voltage pin defined");
  }
}

/********************* LED Lamp Driver **********************/

#define RGB_BITS 24  // WS2812 / SK6812 has 24 bit color in RGB order
static bool lampInit = false;
#if defined(USE_WS2812)
static rmt_data_t ledData[RGB_BITS];
#endif

static void setupLamp() {
  // setup lamp LED according to board type
  // assumes led wired as active high (ESP32 lamp led on pin 4 is active high, signal led on pin 33 is active low)
  lampInit = false;
#if defined(LED_GPIO_NUM)
  if (lampPin <= 0) {
    lampPin = LED_GPIO_NUM;
    char lampPinStr[3];
    sprintf(lampPinStr, "%d", lampPin);
    updateStatus("lampPin", lampPinStr);
  }
#endif

  if (lampPin) {
    lampInit = true;
#if defined(USE_WS2812)
    // WS2812 RGB high intensity led
    if (rmtInit(lampPin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000)) 
      LOG_INF("Setup WS2812 Lamp Led on pin %d", lampPin);
    else {
      LOG_WRN("Failed to setup WS2812 on pin %u", lampPin);
      lampInit = false;
    }
#else
    // assume PWM LED
    ledcAttach(lampPin, 5000, DUTY_BIT_DEPTH); // freq, resolution
    setLamp(0);
    LOG_INF("Setup PWM Lamp Led on pin %d", lampPin);
#endif
  }
  if (lightsRCpin > 1) pinMode(lightsRCpin, OUTPUT);
}

void setLamp(uint8_t lampVal) {
  // control lamp status
  if (lampPin) {
    if (!lampInit) setupLamp();
    if (lampInit) {
#if defined(USE_WS2812)
      // WS2812 LED - set white color and apply lampVal (0 = off, 15 = max)
      uint8_t RGB[3]; // each color is 8 bits
      lampVal = lampVal == 15 ? 255 : lampVal * 16;
      for (uint8_t i = 0; i < 3; i++) {
        RGB[i] = lampVal;
        // apply WS2812 bit encoding pulse timing per bit
        for (uint8_t j = 0; j < 8; j++) { 
          int bit = (i * 8) + j;
          if ((RGB[i] << j) & 0x80) { // get left most bit first
            // bit = 1
            ledData[bit].level0 = 1;
            ledData[bit].duration0 = 8;
            ledData[bit].level1 = 0;
            ledData[bit].duration1 = 4;
          } else {
            // bit = 0
            ledData[bit].level0 = 1;
            ledData[bit].duration0 = 4;
            ledData[bit].level1 = 0;
            ledData[bit].duration1 = 8;
          }
        }
      }
      rmtWrite(lampPin, ledData, RGB_BITS, RMT_WAIT_FOR_EVER);
#else
      // assume PWM LED, set lamp brightness using PWM (0 = off, 15 = max)
      uint8_t valueMax = 15;
      uint32_t duty = (pow(2, DUTY_BIT_DEPTH) / valueMax) * min(lampVal, valueMax);
      ledcWrite(lampPin, duty);
#endif
    }
  }
}

void twinkleLed(uint8_t ledPin, uint16_t interval, uint8_t blinks) {
  // twinkle led, for given number of blinks, 
  //  with given interval in ms between blinks
  bool ledState = true;
  for (int i=0; i<blinks*2; i++) {
    digitalWrite(ledPin, ledState);
    delay(interval);
    ledState = !ledState;
  }
}

void setLightsRC(bool lightsOn) {
  // on / off RC light 
  if (lightsRCpin > 0) digitalWrite(lightsRCpin, lightsOn);
}

static void prepPIR() {
  if (pirUse) {
    if (pirPin) {
      pinMode(pirPin, INPUT_PULLDOWN); // pulled high for active
      if (pirGate) {
        // Attach rising-edge interrupt so pirGateTask can wake the camera.
        attachInterrupt(digitalPinToInterrupt(pirPin), pirGatePIRisr, RISING);
      }
    } else {
      pirUse = false;
      LOG_WRN("No PIR pin defined");
    }
  }
  if (relayPin) pinMode(relayPin, OUTPUT);
}

/********************************* joystick *************************************/

// HW-504 Joystick
// Use X axis  for steering, Y axis for motor, push button for lights toggle
// Requires 2 analog pins and 1 digital pin. Ideally supply voltage should be 3.3V
// X axis is longer edge of board

static const int sRate = 1; // samples per analog reading
static int xOffset = 0; // x zero offset
static int yOffset = 0; // y zero offset
static volatile bool lightsChanged = false;
TaskHandle_t stickHandle = NULL;

static void IRAM_ATTR buttonISR() {
  // joystick button pressed - toggle state
  lightsChanged = !lightsChanged;
}

static void IRAM_ATTR stickISR() {
  // interrupt at timer rate
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (stickHandle) {
    vTaskNotifyGiveFromISR(stickHandle, &xHigherPriorityTaskWoken); 
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void setStickTimer(bool restartTimer, uint32_t interval) {
  // determines joystick polling rate or stepper speed
  static hw_timer_t* stickTimer = NULL;
  // stop timer if running
  if (stickTimer) {
    timerDetachInterrupt(stickTimer); 
    timerEnd(stickTimer);
    stickTimer = NULL;
  }
  if (restartTimer) {
    // (re)start timer interrupt per required interval
    stickTimer = timerBegin(OneMHz); // 1 MHz
    timerAttachInterrupt(stickTimer, &stickISR);
    timerAlarm(stickTimer, interval, true, 0); // in usecs
  }
}

static void stickTask (void *pvParameter) {
  static bool lightsStatus = false;
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (stickUse) {
      // get joystick position, adjusted for zero offset
      int xPos = smoothAnalog(stickXpin, sRate);
      int steerAngle = (xPos > CENTER_ADC + xOffset) ? map(xPos, CENTER_ADC + xOffset, MAX_ADC, servoCenter, servoCenter + maxSteerAngle)
        : map(xPos, 0, CENTER_ADC + xOffset, servoCenter - maxSteerAngle, servoCenter); 
      setSteering(steerAngle);
      
      int yPos = smoothAnalog(stickYpin, sRate);
      // reverse orientation of Y axis so up is forward
      int motorCycle = (yPos > CENTER_ADC + yOffset) ? map(yPos, CENTER_ADC + yOffset, MAX_ADC, 0, 0 - maxDutyCycle)
        : map(yPos, 0, CENTER_ADC + yOffset, maxDutyCycle, 0); 
      if (abs(motorCycle) < minDutyCycle) motorCycle = 0; // deadzone
#if INCLUDE_MCPWM
      motorSpeed(motorCycle);
#endif
      if (lightsChanged != lightsStatus) setLightsRC(lightsChanged);
      lightsStatus = lightsChanged;
      LOG_VRB("Xpos %d, Ypos %d, button %d", xPos, yPos, lightsStatus);
    }
  }
}

static void prepJoystick() {
  if (stickUse) {
    if (stickXpin > 0 && stickYpin > 0) {
      // obtain offsets at joystick resting position
      xOffset = smoothAnalog(stickXpin, 8) - CENTER_ADC;
      yOffset = smoothAnalog(stickYpin, 8) - CENTER_ADC;
      LOG_VRB("X-offset: %d, Y-offset: %d", xOffset, yOffset);
      if (stickzPushPin > 0) {
        pinMode(stickzPushPin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(stickzPushPin), buttonISR, FALLING); 
      }
      if (stickHandle == NULL) xTaskCreateWithCaps(&stickTask, "stickTask", STICK_STACK_SIZE , NULL, STICK_PRI, &stickHandle, STACK_MEM);
      setStickTimer(true, waitTime * 1000);
      LOG_INF("Joystick available");
    } else {
      stickUse = false;
      LOG_WRN("Joystick pins not defined");
    }
  }
}

/****************************** stepper motors *************************************/

// Unipolar 28BYJ-48 Geared Stepper Motor with ULN2003 Motor Driver
// Bipolar generic using MX1508 H-Bridge Motor Driver
// For 4 pin stepper motor drivers in full step mode
// Supports two independent steppers (index 0 = pan, 1 = tilt), each with its own
// hardware timer and FreeRTOS task so they can run simultaneously at different
// speeds and directions.

#define stepPhases 4
#define modelTypes 2 // must equal enum stepperModel entries in appGlobals.h
static const uint16_t stepsPerRevolution[modelTypes] = {32 * 64, 20}; // number of steps in geared 28BYJ-48 unipolar, 8mm bipolar
static const uint8_t pinSequence[stepPhases * modelTypes][stepperPins] = {
  // 28BYJ-48 unipolar full step phases
  {1, 1, 0, 0}, 
  {0, 1, 1, 0}, 
  {0, 0, 1, 1}, 
  {1, 0, 0, 1},
  // 8mm bipolar half step phases
  {1, 0, 1, 0}, 
  {0, 1, 1, 0}, 
  {0, 1, 0, 1}, 
  {1, 0, 0, 1}, 
};

// Per-stepper runtime state (independent so both can run at once)
typedef struct {
  TaskHandle_t taskHandle;   // stepper task handle
  hw_timer_t* hwTimer;        // hardware timer driving this stepper
  volatile uint32_t stepsToDo; // remaining steps requested
  uint8_t modelIndex;         // base index into pinSequence (stepPhases * model)
  uint8_t stepPhase;          // current phase 0..stepPhases-1
  bool clockwise;             // current rotation direction
  bool running;               // true while a sequence is active
  int32_t currentPos;         // accumulated position in steps (signed, for step tracking)
  uint8_t targetPct;          // last commanded target position 0~100% (for HA cover feedback)
  stepperModel model;         // stepper model for position conversion
} stepperState_t;

static stepperState_t stepper[stepperCount] = {0};
static SemaphoreHandle_t stepperTimerMutex = NULL;

static void IRAM_ATTR stepperISR0();
static void IRAM_ATTR stepperISR1();
static void stepperTask0(void* pvParameters);
static void stepperTask1(void* pvParameters);

static void (*stepperISR[stepperCount])() = {stepperISR0, stepperISR1};
static void (*stepperTask[stepperCount])(void*) = {stepperTask0, stepperTask1};
static const char* stepperName[stepperCount] = {"pan", "tilt"};

void setStepperPin(uint8_t stepperIdx, uint8_t pinNum, uint8_t pinPos) {
  // Pin order is IN1, IN2, IN3, IN4 for correct full stepping
  // 28BYJ-48 wire color order: blue, pink, yellow, orange, red from driver not motor
  // bipolar wire order: A+, A-, B+, B-
  if (stepperIdx < stepperCount && pinPos < stepperPins) stepINpins[stepperIdx][pinPos] = pinNum;
}

static void nextPhase(uint8_t idx, bool changeDir = false) {
  // identify next phase for given stepper
  if (changeDir) stepper[idx].clockwise = !stepper[idx].clockwise;
  if (stepper[idx].clockwise) stepper[idx].stepPhase = (stepper[idx].stepPhase == 0) ? stepPhases - 1 : stepper[idx].stepPhase - 1;
  else if (++stepper[idx].stepPhase >= stepPhases) stepper[idx].stepPhase = 0;
}

static void setStepperTimer(uint8_t idx, bool restartTimer, uint32_t interval) {
  // (re)start or stop the hardware timer for the given stepper
  // protected by mutex to prevent timer double-free race between doStep
  // (stepperTask) and stepperRun/stepperStop (httpd/captureTask)
  if (stepperTimerMutex != NULL && xSemaphoreTake(stepperTimerMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    LOG_WRN("setStepperTimer %u mutex timeout", idx);
    return;
  }
  if (stepper[idx].hwTimer) {
    timerDetachInterrupt(stepper[idx].hwTimer);
    timerEnd(stepper[idx].hwTimer);
    stepper[idx].hwTimer = NULL;
  }
  if (restartTimer) {
    stepper[idx].hwTimer = timerBegin(OneMHz); // 1 MHz
    if (stepper[idx].hwTimer == NULL) {
      LOG_WRN("Stepper %u timerBegin failed (no free timers)", idx);
    } else {
      timerAttachInterrupt(stepper[idx].hwTimer, stepperISR[idx]);
      timerAlarm(stepper[idx].hwTimer, interval, true, 0); // in usecs
    }
  }
  if (stepperTimerMutex != NULL) xSemaphoreGive(stepperTimerMutex);
}

static void doStep(uint8_t idx) {
  // called from stepper task to do single step in sequence
  if (stepper[idx].stepsToDo > 0) {
    stepper[idx].stepsToDo--;
#if defined(PCF8575_ADDR)
    if (usePCF8575) {
      uint8_t portIdx = idx; // stepper 0 -> port 0, stepper 1 -> port 1
      uint8_t val = 0;
      for (int i = 0; i < stepperPins; i++)
        val |= (pinSequence[stepper[idx].modelIndex + stepper[idx].stepPhase][i] << i);
      pcf8575WritePort(portIdx, val, 0x0F); // only lower nibble, preserve upper nibble for aux lights
    } else
#endif
    {
      for (int i = 0; i < stepperPins; i++) digitalWrite(stepINpins[idx][i], pinSequence[stepper[idx].modelIndex + stepper[idx].stepPhase][i]);
    }
    nextPhase(idx);
    // track accumulated position (signed steps) for HA cover feedback
    stepper[idx].currentPos += stepper[idx].clockwise ? 1 : -1;
  } else {
    // step sequence completed
    setStepperTimer(idx, false, 0); // stop this stepper's timer
#if defined(PCF8575_ADDR)
    if (usePCF8575) {
      pcf8575WritePort(idx, 0x00, 0x0F); // only clear lower nibble, preserve upper nibble for aux lights
    } else
#endif
    {
      for (int i = 0; i < stepperPins; i++) digitalWrite(stepINpins[idx][i], LOW); // stop unnecessary power use
    }
    stepper[idx].running = false;
    LOG_INF("stepper %u done: pos=%d pct=%u", idx, (int)stepper[idx].currentPos, stepperGetPosition(idx));
#if (INCLUDE_PGRAM && INCLUDE_PERIPH)
    if (idx == 0) stepperDone(); // photogrammetry uses stepper 0
#endif
  }
}

static void IRAM_ATTR stepperISR0() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (stepper[0].taskHandle) vTaskNotifyGiveFromISR(stepper[0].taskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
static void IRAM_ATTR stepperISR1() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if (stepper[1].taskHandle) vTaskNotifyGiveFromISR(stepper[1].taskHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void stepperTask0(void* pvParameters) {
  while (true) { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); doStep(0); }
}
static void stepperTask1(void* pvParameters) {
  while (true) { ulTaskNotifyTake(pdTRUE, portMAX_DELAY); doStep(1); }
}

static void prepStepper() {
  if (stepperUse) {
    if (stepperTimerMutex == NULL) stepperTimerMutex = xSemaphoreCreateMutex();
#if defined(PCF8575_ADDR)
    pcf8575Init();
    // auto-configure stepper pins on PCF8575: P00~P03 for pan, P10~P13 for tilt
    for (uint8_t idx = 0; idx < stepperCount; idx++) {
      for (uint8_t i = 0; i < stepperPins; i++) {
        // use pin numbers > 100 to indicate PCF8575 pins (100+port*8+bit)
        stepINpins[idx][i] = 100 + idx * 8 + i;
      }
    }
#endif
    bool anyOk = false;
    for (uint8_t idx = 0; idx < stepperCount; idx++) {
      if (stepINpins[idx][0] > 0 && stepINpins[idx][1] > 0) {
        stepper[idx].stepPhase = 0;
#if defined(PCF8575_ADDR)
        if (!usePCF8575)
#endif
        {
          for (int i = 0; i < stepperPins; i++) {
            pinMode(stepINpins[idx][i], OUTPUT);
            digitalWrite(stepINpins[idx][i], LOW);
          }
        }
        if (stepper[idx].taskHandle == NULL)
          xTaskCreateWithCaps(stepperTask[idx], stepperName[idx], STICK_STACK_SIZE, NULL, STICK_PRI, &stepper[idx].taskHandle, STACK_MEM);
        stepper[idx].model = BYJ_48;
        stepperSetPosition(idx, 50);
        stepperSetSteps(idx, stepsPerRevolution[BYJ_48] / 2); // currentPos = 50% of a rev
#if defined(PCF8575_ADDR)
        if (usePCF8575)
          LOG_INF("Stepper %u (%s) on PCF8575 port %u pins P%d0~P%d3", idx, stepperName[idx], idx, idx, idx);
        else
#endif
          LOG_INF("Stepper %u (%s) on pins: %d, %d, %d, %d", idx, stepperName[idx],
            stepINpins[idx][0], stepINpins[idx][1], stepINpins[idx][2], stepINpins[idx][3]);
        anyOk = true;
      } else LOG_WRN("Stepper %u (%s) pins not defined", idx, stepperName[idx]);
    }
    if (!anyOk) stepperUse = false;
    else {
      if (!servoPanPin && stepINpins[0][0] > 0) { servoPanPin = 1; updateConfigVect("servoPanPin", "1"); }
      if (!servoTiltPin && stepINpins[1][0] > 0) { servoTiltPin = 1; updateConfigVect("servoTiltPin", "1"); }
    }
  }
}

void stepperRun(uint8_t idx, float RPM, float revFraction, bool _clockwise, stepperModel thisStepper) {
  // idx is stepper index (0 = pan, 1 = tilt)
  // RPM is stepper motor rotation speed
  // revFraction is required movement as a fraction of full rotation (sign ignored, use _clockwise for direction)
  // thisStepper is stepper model type to determine steps per revolution
  if (idx >= stepperCount || (stepINpins[idx][0] <= 0
#if defined(PCF8575_ADDR)
    && !usePCF8575
#endif
  )) {
    LOG_WRN("Stepper %u not available", idx);
    return;
  }
  if (RPM <= 0) {
    LOG_WRN("Stepper %u RPM must be > 0", idx);
    return;
  }
  stepper[idx].stepsToDo = (uint32_t)(fabsf(revFraction) * stepsPerRevolution[thisStepper]);
  stepper[idx].modelIndex = stepPhases * thisStepper;
  stepper[idx].model = thisStepper;
  if (stepper[idx].clockwise != _clockwise) {
    // change of direction, modify next phase to be in reversed sequence
    nextPhase(idx, true);
    nextPhase(idx);
  }
  uint32_t stepDelay = 60 * USECS / RPM; // duration of 1 rev in microsecs
  stepDelay /= stepsPerRevolution[thisStepper]; // duration per step

  // stop previous timer; if mutex unavailable, abort to avoid
  // leaving running=true without an active timer
  if (stepperTimerMutex != NULL && xSemaphoreTake(stepperTimerMutex, pdMS_TO_TICKS(100)) != pdTRUE) {
    LOG_WRN("stepperRun %u: mutex busy, aborting", idx);
    return;
  }
  if (stepper[idx].hwTimer) {
    timerDetachInterrupt(stepper[idx].hwTimer);
    timerEnd(stepper[idx].hwTimer);
    stepper[idx].hwTimer = NULL;
  }
  // start this stepper's timer
  stepper[idx].hwTimer = timerBegin(OneMHz); // 1 MHz
  if (stepper[idx].hwTimer == NULL) {
    LOG_WRN("Stepper %u timerBegin failed (no free timers)", idx);
    if (stepperTimerMutex != NULL) xSemaphoreGive(stepperTimerMutex);
    return;
  }
  timerAttachInterrupt(stepper[idx].hwTimer, stepperISR[idx]);
  timerAlarm(stepper[idx].hwTimer, stepDelay, true, 0); // in usecs
  stepper[idx].running = true;
  if (stepperTimerMutex != NULL) xSemaphoreGive(stepperTimerMutex);
  LOG_INF("stepperRun %u: steps=%u cw=%d delay=%uus", idx, (unsigned)stepper[idx].stepsToDo, _clockwise, (unsigned)stepDelay);
}

void stepperStop(uint8_t idx) {
  // immediately stop a given stepper
  if (idx >= stepperCount) return;
  setStepperTimer(idx, false, 0);
#if defined(PCF8575_ADDR)
  if (usePCF8575) {
    pcf8575WritePort(idx, 0x00, 0x0F); // only clear lower nibble, preserve upper nibble for aux lights
  } else
#endif
  {
    for (int i = 0; i < stepperPins; i++) digitalWrite(stepINpins[idx][i], LOW);
  }
  stepper[idx].stepsToDo = 0;
  stepper[idx].running = false;
}

bool stepperIsRunning(uint8_t idx) {
  return (idx < stepperCount) ? stepper[idx].running : false;
}

uint16_t stepperStepsPerRev(stepperModel thisStepper) {
  // expose steps per revolution for a given stepper model
  return (thisStepper < modelTypes) ? stepsPerRevolution[thisStepper] : 0;
}

uint8_t stepperGetPosition(uint8_t idx) {
  // return last commanded target position as 0~100% (for HA cover feedback)
  // uses targetPct rather than currentPos%stepsPerRev to avoid 100% wrapping to 0
  if (idx >= stepperCount) return 0;
  return stepper[idx].targetPct;
}

int32_t stepperGetSteps(uint8_t idx) {
  // return raw signed step count (for calibration that needs finer than 1%)
  if (idx >= stepperCount) return 0;
  return stepper[idx].currentPos;
}

void stepperSetSteps(uint8_t idx, int32_t steps) {
  // set raw step count without moving (used by calibration)
  if (idx >= stepperCount) return;
  stepper[idx].currentPos = steps;
}

void stepperSetPosition(uint8_t idx, uint8_t percent) {
  // set target position reference (0~100%); used to calibrate / reset and to
  // record the commanded target so HA cover feedback stays consistent.
  // NOTE: does NOT modify currentPos (the accumulated step count) so that
  // panHome/tiltHome calibration and panPos/tiltPos diff math stay consistent.
  if (idx >= stepperCount) return;
  uint8_t pct = (percent > 100) ? 100 : percent;
  stepper[idx].targetPct = pct;
}

/******************* MY9221 LED Bar ***************************/
/*
 LED segment bar with MY9221 LED driver, eg Grove LED Bar
 Wiring:
    Black  GND
    Red    3V3
    White  DCKI Clock pin
    Yellow D1 Data pin
    
 Can be used as a gauge, eg display sound level
 */

#define MY9221_COUNT 12 // max number of leds addressable by MY9221 LED driver
#define LEDBAR_COUNT 10 // number of leds in bar display
#define LED_OFF 0x00
#define LED_FULL 0xFF

static bool reverse = true; // from which end to light leds, true is green -> red on Grove LED Bar
static uint8_t ledLevel[LEDBAR_COUNT];

static void ledBarLatch() {
  // display uploaded register by triggering internal-latch function
  digitalWrite(ledBarClock, LOW); 
  delayMicroseconds(250); // minimum 220us
  // Internal-latch control cycle
  bool dataVal = false;
  for (uint8_t i = 0; i < 8; i++, dataVal = !dataVal) {
    digitalWrite(ledBarData, dataVal ? HIGH : LOW);
    delayMicroseconds(1); // > min pulse length 230ns 
  }
}

static void ledBarSend(uint16_t bits) {
  // output led value as clocked 16 bits (only 8 LSB set for 8 bit greyscale)
  bool clockVal = false;
  for (int i = 15; i >= 0; i--, clockVal = !clockVal) {
    digitalWrite(ledBarData, (bits >> i) & 1 ? HIGH : LOW);
    digitalWrite(ledBarClock, clockVal ? HIGH : LOW);
  }
}

void ledBarClear() {
  for (uint8_t i = 0; i < LEDBAR_COUNT; i++) ledLevel[i] = LED_OFF;
}

void ledBrightness(uint8_t whichLed, float brightness) {
  // brightness is a float 0.0 <> 1.0, converted to one of 8 brightness levels or off
  ledLevel[whichLed] |= (1 << (uint8_t)(8 * brightness)) - 1;
}

void ledBarUpdate() {
  // update MY9221 208 bit register with required values
  if (ledBarUse) {
    ledBarSend(0); // initial 16 bit command, as 8 bit greyscale mode + defaults
    // 12 * 16 bits LED greyscale PWM values
    for (uint8_t i = 0; i < LEDBAR_COUNT; i++) // 10 * 16 bits
      ledBarSend(reverse ? ledLevel[LEDBAR_COUNT - 1 - i] : ledLevel[i]);
    // fill register for remaining unused channels
    for (uint8_t i = 0; i < MY9221_COUNT - LEDBAR_COUNT; i++) ledBarSend(LED_OFF);
    ledBarLatch();
  }
}
       
void ledBarGauge(float level) {
  // set how many leds to be switched on and their brightness
  // as a proportion of level between 0.0 and 1.0
  // least significant leds are full brightness and most significant led
  // has a proportional brightness
  level = fabsf(level);
  if (ledBarUse) {
    ledBarClear();
    uint8_t fullLedCnt = min((uint8_t)(level * LEDBAR_COUNT), (uint8_t)(LEDBAR_COUNT - 1));
    for (uint8_t i = 0; i < fullLedCnt; i++) ledLevel[i] = LED_FULL;
    // set brightness for most significant lit led
    ledBrightness(fullLedCnt, (LEDBAR_COUNT * level) - fullLedCnt); 
    ledBarUpdate();
  }
}

static void prepLedBar() {
  // initialise led state and setup pins
  if (ledBarUse && ledBarClock && ledBarData) {
    pinMode(ledBarClock, OUTPUT);
    pinMode(ledBarData, OUTPUT);
    ledBarClear();
    ledBarUpdate();
    LOG_INF("Setup %d Led Bar with pins %d, %d", LEDBAR_COUNT, ledBarClock, ledBarData);
  } else ledBarUse = false;
}

/**********************************************/

static void prepAuxLights() {
  // setup 3 auxiliary lights on PCF8575 (manual web control only)
#if defined(PCF8575_ADDR)
  if (usePCF8575) {
    // ensure all aux lights off at startup
    for (uint8_t i = 0; i < 3; i++) {
      auxLightOn[i] = false;
      if (auxLightPin[i] <= 15) pcf8575WritePin(auxLightPin[i], false);
    }
    LOG_INF("Aux lights on PCF8575 P%u/P%u/P%u (white/red/IR)", auxLightPin[0], auxLightPin[1], auxLightPin[2]);
  } else LOG_WRN("PCF8575 not available, aux lights disabled");
#else
  LOG_WRN("PCF8575 not defined, aux lights disabled");
#endif
}

/**********************************************/

void prepPeripherals() {
  // initial setup of each peripheral on client or extender
  setupADC();
  setupBatt();
  setupLamp();
  prepPIR();
  prepTemperature();
  prepServos();  
  prepJoystick();
  prepStepper();
  prepLedBar();
  prepAuxLights();
  debugMemory("prepPeripherals");
}

#endif
