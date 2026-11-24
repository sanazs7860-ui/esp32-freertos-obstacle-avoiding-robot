/*
  RTOS obstacle-avoiding robot: ESP32 (30-pin) + L298N + HC-SR04 + MPU-6050 + Wheel Encoders.

  Pins:
    L298N IN1 -> D26   IN2 -> D27   (left pair)
    L298N IN3 -> D32   IN4 -> D33   (right pair)
    HC-SR04 TRIG -> D18, ECHO -> D19 (through the voltage divider)
    MPU-6050 SDA -> D21, SCL -> D22 (AD0 -> GND, address 0x68)
    Left Encoder  DO -> D16 (RX2)
    Right Encoder DO -> D17 (TX2)
    Onboard LED (D2) = heartbeat

  Tasks (higher number = higher priority):
    motorTask  (5): owns the motors, watchdog + emergency override       (Core 1)
    imuTask    (4): 100 Hz MPU-6050 yaw integration via Wire.h           (Core 1)
    sensorTask (3): periodic ultrasonic read, 50 ms                      (Core 1)
    navTask    (2): state machine DRIVE / BACKUP / IMU-guided TURN       (Core 1)
    statusTask (1): prints diagnostics every 2 s (USB serial)            (Core 0)
    bleTask    (1): sends live data to the phone over BLE                (Core 0)
*/

#include <stdarg.h>
#include <math.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "freertos/timers.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- Pins ----------------
#define L_IN1      26
#define L_IN2      27
#define R_IN1      32
#define R_IN2      33
#define TRIG_PIN   18
#define ECHO_PIN   19
#define ENC_L_PIN  16   // RX2
#define ENC_R_PIN  17   // TX2
#define I2C_SDA    21
#define I2C_SCL    22
#define LED_PIN    2

#define MPU_ADDR   0x68

// ---------------- Settings you can tune ----------------
const bool LEFT_INVERT  = false;   // flip if a whole side runs backward
const bool RIGHT_INVERT = true;

const int   DRIVE_SPEED   = 170;   // 0..255
const int   TURN_SPEED    = 255;
const float EMERGENCY_CM  = 15;    // safety layer: no forward motion closer than this
const float STOP_DIST_CM  = 25;    // navigation stops here
const float CLEAR_DIST_CM = 45;    // fallback clear distance if IMU is disconnected
const float MAX_DIST_CM   = 200;

const float TURN_ANGLE_DEG        = 90.0f;   // exact turn angle using MPU-6050
const float TURN_AROUND_ANGLE_DEG = 180.0f;  // exact U-turn when stuck

const int IMU_PERIOD_MS    = 10;   // 100 Hz
const int SENSOR_PERIOD_MS = 50;   // 20 Hz
const int CMD_TIMEOUT_MS   = 300;  // watchdog: no command for this long -> stop

const int BACKUP_MS          = 300;
const int MIN_TURN_MS        = 250;
const int MAX_TURN_MS        = 1800;
const int MAX_TURN_AROUND_MS = 3000;

const bool DEMO_BLIND_NAV = false;

// ---- Bluetooth settings ----
const char *BLE_NAME      = "RoboCar";
const int   BLE_UPDATE_MS = 500;

// ---------------- PWM helpers (old and new ESP32 cores) ----------------
#if ESP_ARDUINO_VERSION_MAJOR < 3
int chanOfPin[40];
int nextChan = 0;
#endif

void pwmAttach(int pin, int freq, int res) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, freq, res);
#else
  ledcSetup(nextChan, freq, res);
  ledcAttachPin(pin, nextChan);
  chanOfPin[pin] = nextChan++;
#endif
}

void pwmWrite(int pin, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(chanOfPin[pin], duty);
#endif
}

// ---------------- Motors (only motorTask calls these) ----------------
void setMotor(int in1, int in2, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0)      { pwmWrite(in1, speed); pwmWrite(in2, 0); }
  else if (speed < 0) { pwmWrite(in1, 0);     pwmWrite(in2, -speed); }
  else                { pwmWrite(in1, 0);     pwmWrite(in2, 0); }
}

void setSides(int left, int right) {
  if (LEFT_INVERT)  left  = -left;
  if (RIGHT_INVERT) right = -right;
  setMotor(L_IN1, L_IN2, left);
  setMotor(R_IN1, R_IN2, right);
}

// ---------------- Wheel Encoders (Hardware Interrupts) ----------------
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
volatile long     encLeftTicks  = 0;
volatile long     encRightTicks = 0;
volatile uint32_t lastEncLUs    = 0;
volatile uint32_t lastEncRUs    = 0;

void IRAM_ATTR encLeftISR() {
  uint32_t now = micros();
  if (now - lastEncLUs > 1500) {   // 1.5 ms glitch filter for optical sensor
    portENTER_CRITICAL_ISR(&encMux);
    encLeftTicks++;
    portEXIT_CRITICAL_ISR(&encMux);
    lastEncLUs = now;
  }
}

void IRAM_ATTR encRightISR() {
  uint32_t now = micros();
  if (now - lastEncRUs > 1500) {
    portENTER_CRITICAL_ISR(&encMux);
    encRightTicks++;
    portEXIT_CRITICAL_ISR(&encMux);
    lastEncRUs = now;
  }
}

void getEncoders(long &l, long &r) {
  portENTER_CRITICAL(&encMux);
  l = encLeftTicks;
  r = encRightTicks;
  portEXIT_CRITICAL(&encMux);
}

// ---------------- MPU-6050 Direct Wire.h Helpers ----------------
bool  imuOk      = false;
float gyroZBias  = 0.0f;

bool mpuInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);           // 100 kHz for clean signals on jumper wires

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);                // PWR_MGMT_1
  Wire.write(0x00);                // wake up
  if (Wire.endTransmission(true) != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                // GYRO_CONFIG
  Wire.write(0x00);                // +/- 250 deg/s (131 LSB / deg/s)
  if (Wire.endTransmission(true) != 0) return false;

  return true;
}

bool mpuReadGyroZ(int16_t &gzRaw) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2, (uint8_t)true) != 2) return false;
  gzRaw = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

void calibrateGyroZ() {
  long sum = 0;
  int  valid = 0;
  for (int i = 0; i < 200; i++) {
    int16_t gz;
    if (mpuReadGyroZ(gz)) {
      sum += gz;
      valid++;
    }
    delay(5);
  }
  if (valid > 0) gyroZBias = (float)sum / (float)valid;
}

// ---------------- Ultrasonic ----------------
float readOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long d = pulseIn(ECHO_PIN, HIGH, 12000);
  if (d == 0) return MAX_DIST_CM;
  return min(d * 0.0343f / 2.0f, MAX_DIST_CM);
}

float readMedian3() {
  float a = readOnce(); delay(3);
  float b = readOnce(); delay(3);
  float c = readOnce();
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// ---------------- Shared RTOS objects ----------------
struct MotorCmd { int16_t left; int16_t right; };

QueueHandle_t      distQ;    // sensor -> nav/status/ble: latest distance (float)
QueueHandle_t      yawQ;     // imu    -> nav/status/ble: latest yaw in deg (float)
QueueHandle_t      cmdQ;     // nav    -> motor: latest command (MotorCmd)
EventGroupHandle_t evt;      // safety flags
SemaphoreHandle_t  logMtx;   // protects Serial

const EventBits_t EVT_EMERGENCY = (1 << 0);

TaskHandle_t hMotor, hImu, hSensor, hNav, hStatus;

volatile uint32_t emergencyStampUs = 0;
volatile uint32_t maxLatencyUs     = 0;
volatile int      gState           = 0;

const char *stateNames[] = { "DRIVE", "BACKUP", "TURN" };

void logf(const char *fmt, ...) {
  char buf[180];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (xSemaphoreTake(logMtx, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.print(buf);
    xSemaphoreGive(logMtx);
  }
}

void sendCmd(int l, int r) {
  MotorCmd c = { (int16_t)l, (int16_t)r };
  xQueueOverwrite(cmdQ, &c);
}

void holdCmd(int l, int r, int ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < (uint32_t)ms) {
    sendCmd(l, r);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

float getLatestYaw() {
  float y = 0.0f;
  xQueuePeek(yawQ, &y, 0);
  return y;
}

// ---------------- Motor task (priority 5) ----------------
void motorTask(void *pv) {
  MotorCmd cmd = { 0, 0 };
  uint32_t lastCmdMs = millis();
  bool overriding = false;

  for (;;) {
    MotorCmd n;
    if (xQueueReceive(cmdQ, &n, pdMS_TO_TICKS(10)) == pdTRUE) {
      cmd = n;
      lastCmdMs = millis();
    }

    if (millis() - lastCmdMs > (uint32_t)CMD_TIMEOUT_MS) {
      cmd.left = 0;
      cmd.right = 0;
    }

    EventBits_t bits = xEventGroupGetBits(evt);
    bool forward  = (cmd.left > 0 && cmd.right > 0);
    bool override = (bits & EVT_EMERGENCY) && forward;

    if (override) {
      setSides(0, 0);
      if (!overriding) {
        overriding = true;
        uint32_t lat = micros() - emergencyStampUs;
        if (lat < 500000UL && lat > maxLatencyUs) maxLatencyUs = lat;
      }
    } else {
      overriding = false;
      setSides(cmd.left, cmd.right);
    }
  }
}

// ---------------- IMU task (priority 4, 100 Hz fixed period) ----------------
void imuTask(void *pv) {
  TickType_t last = xTaskGetTickCount();
  uint32_t prevUs = micros();
  float yawDeg = 0.0f;
  xQueueOverwrite(yawQ, &yawDeg);

  for (;;) {
    uint32_t nowUs = micros();
    float dt = (nowUs - prevUs) * 1e-6f;
    prevUs = nowUs;

    if (imuOk) {
      int16_t gzRaw;
      if (mpuReadGyroZ(gzRaw)) {
        float gzDegPerSec = (gzRaw - gyroZBias) / 131.0f;
        if (fabsf(gzDegPerSec) < 0.6f) gzDegPerSec = 0.0f;   // deadband to stop idle drift
        yawDeg += gzDegPerSec * dt;
        xQueueOverwrite(yawQ, &yawDeg);
      }
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(IMU_PERIOD_MS));
  }
}

// ---------------- Sensor task (priority 3, 20 Hz fixed period) ----------------
void sensorTask(void *pv) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    float d = readMedian3();
    xQueueOverwrite(distQ, &d);

    if (d < EMERGENCY_CM) {
      if (!(xEventGroupGetBits(evt) & EVT_EMERGENCY)) {
        emergencyStampUs = micros();
        xEventGroupSetBits(evt, EVT_EMERGENCY);
      }
    } else {
      xEventGroupClearBits(evt, EVT_EMERGENCY);
    }
    vTaskDelayUntil(&last, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
  }
}

// ---------------- Navigation task (priority 2) ----------------
enum State { DRIVE = 0, BACKUP = 1, TURN = 2 };

void navTask(void *pv) {
  State s = DRIVE;
  int  closeCount  = 0;
  int  recentCount = 0;
  uint32_t lastAvoidMs = 0;

  for (;;) {
    gState = s;
    switch (s) {

      case DRIVE: {
        if (DEMO_BLIND_NAV) {
          sendCmd(DRIVE_SPEED, DRIVE_SPEED);
          vTaskDelay(pdMS_TO_TICKS(20));
          break;
        }
        float d;
        if (xQueuePeek(distQ, &d, pdMS_TO_TICKS(200)) != pdTRUE) {
          sendCmd(0, 0);
          break;
        }
        if (d <= STOP_DIST_CM) closeCount++;
        else closeCount = 0;

        if (closeCount >= 2) {
          sendCmd(0, 0);
          logf("Obstacle at %.0f cm\n", d);
          closeCount = 0;
          s = BACKUP;
        } else {
          sendCmd(DRIVE_SPEED, DRIVE_SPEED);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        break;
      }

      case BACKUP: {
        holdCmd(-DRIVE_SPEED, -DRIVE_SPEED, BACKUP_MS);
        sendCmd(0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        s = TURN;
        break;
      }

      case TURN: {
        uint32_t now = millis();
        if (now - lastAvoidMs < 4000) recentCount++;
        else recentCount = 1;
        lastAvoidMs = now;

        bool turnAround = (recentCount >= 3);
        if (turnAround) recentCount = 0;

        int dir = (esp_random() & 1) ? 1 : -1;
        int l = (dir > 0) ?  TURN_SPEED : -TURN_SPEED;
        int r = (dir > 0) ? -TURN_SPEED :  TURN_SPEED;

        float targetAngle = turnAround ? TURN_AROUND_ANGLE_DEG : TURN_ANGLE_DEG;
        float startYaw    = getLatestYaw();
        logf("Turning %s (target %.0f deg)\n", dir > 0 ? "right" : "left", targetAngle);

        xQueueReset(distQ);
        uint32_t t0    = millis();
        uint32_t maxMs = turnAround ? MAX_TURN_AROUND_MS : MAX_TURN_MS;
        int  clearCount = 0;
        bool reached    = false;

        while (millis() - t0 < maxMs) {
          sendCmd(l, r);
          vTaskDelay(pdMS_TO_TICKS(20));

          if (imuOk) {
            float turned = fabsf(getLatestYaw() - startYaw);
            if (turned >= targetAngle) {
              reached = true;
              break;
            }
          } else {
            // Fallback if IMU is disconnected: turn until ultrasonic sees clear path
            float d;
            if (xQueuePeek(distQ, &d, 0) == pdTRUE) {
              if (millis() - t0 > (uint32_t)MIN_TURN_MS && d > CLEAR_DIST_CM) clearCount++;
              else clearCount = 0;
              if (clearCount >= 2) { reached = true; break; }
            }
          }
        }
        sendCmd(0, 0);
        logf("Turn done after %lu ms | yaw=%.1f (%s)\n",
             (unsigned long)(millis() - t0), getLatestYaw(),
             reached ? "target reached" : "timeout");
        vTaskDelay(pdMS_TO_TICKS(100));
        xQueueReset(distQ);
        s = DRIVE;
        break;
      }
    }
  }
}

// ---------------- Status task (priority 1, core 0) ----------------
void statusTask(void *pv) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(2000));
    float d = -1;
    xQueuePeek(distQ, &d, 0);
    float y = getLatestYaw();
    long encL, encR;
    getEncoders(encL, encR);

    logf("dist %.0fcm | yaw %.1fdeg | enc L:%ld R:%ld | state %s | maxLat %luus | stack M:%u I:%u S:%u N:%u\n",
         d, y, encL, encR, stateNames[gState], (unsigned long)maxLatencyUs,
         (unsigned)uxTaskGetStackHighWaterMark(hMotor),
         (unsigned)uxTaskGetStackHighWaterMark(hImu),
         (unsigned)uxTaskGetStackHighWaterMark(hSensor),
         (unsigned)uxTaskGetStackHighWaterMark(hNav));
  }
}

// ---------------- Heartbeat software timer ----------------
void heartbeatCb(TimerHandle_t t) {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// ---------------- Bluetooth Low Energy (priority 1, core 0) ----------------
#define NUS_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer         *bleServer = nullptr;
BLECharacteristic *bleTx     = nullptr;
volatile bool      bleConnected = false;

class BleCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s)    { bleConnected = true; }
  void onDisconnect(BLEServer *s) { bleConnected = false; }
};

void bleSend(const char *msg) {
  bleTx->setValue((uint8_t *)msg, strlen(msg));
  bleTx->notify();
}

void bleTask(void *pv) {
  BLEDevice::init(BLE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_N0);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new BleCallbacks());

  BLEService *svc = bleServer->createService(NUS_SERVICE_UUID);
  bleTx = svc->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  svc->createCharacteristic(NUS_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(NUS_SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
  logf("Bluetooth started: look for '%s'\n", BLE_NAME);

  bool wasConnected = false;
  int  tickCount = 0;

  for (;;) {
    if (bleConnected) {
      float d = -1;
      xQueuePeek(distQ, &d, 0);
      int em = (xEventGroupGetBits(evt) & EVT_EMERGENCY) ? 1 : 0;

      char buf[24];
      snprintf(buf, sizeof(buf), "%3.0fcm %s E%d\n", d, stateNames[gState], em);
      bleSend(buf);

      tickCount++;
      if (tickCount == 2) {
        long encL, encR;
        getEncoders(encL, encR);
        snprintf(buf, sizeof(buf), "Y%.0f L%ld R%ld\n", getLatestYaw(), encL, encR);
        bleSend(buf);
      } else if (tickCount >= 4) {
        tickCount = 0;
        snprintf(buf, sizeof(buf), "maxLat %luus\n", (unsigned long)maxLatencyUs);
        bleSend(buf);
      }
    } else if (wasConnected) {
      vTaskDelay(pdMS_TO_TICKS(300));
      adv->start();
    }
    wasConnected = bleConnected;
    vTaskDelay(pdMS_TO_TICKS(BLE_UPDATE_MS));
  }
}

// ---------------- Setup ----------------
void setup() {
  setCpuFrequencyMhz(80);
  Serial.begin(115200);
  delay(300);
  logMtx = xSemaphoreCreateMutex();
  logf("Reset reason: %d\n", (int)esp_reset_reason());

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(ENC_L_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), encLeftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), encRightISR, RISING);

  pwmAttach(L_IN1, 1000, 8);
  pwmAttach(L_IN2, 1000, 8);
  pwmAttach(R_IN1, 1000, 8);
  pwmAttach(R_IN2, 1000, 8);
  setSides(0, 0);

  distQ = xQueueCreate(1, sizeof(float));
  yawQ  = xQueueCreate(1, sizeof(float));
  cmdQ  = xQueueCreate(1, sizeof(MotorCmd));
  evt   = xEventGroupCreate();

  // Start BLE while motors are idle
  xTaskCreatePinnedToCore(bleTask, "ble", 8192, NULL, 1, NULL, 0);

  // Initialize and calibrate MPU-6050 while robot sits completely still
  logf("Keep robot still: calibrating MPU-6050...\n");
  imuOk = mpuInit();
  if (imuOk) {
    calibrateGyroZ();
    logf("MPU-6050 ready (gyroZBias = %.2f)\n", gyroZBias);
  } else {
    logf("MPU-6050 not detected — falling back to ultrasonic turn mode.\n");
  }

  logf("Starting in 4 seconds.\n");
  delay(4000);

  xTaskCreatePinnedToCore(motorTask,  "motor",  4096, NULL, 5, &hMotor,  1);
  xTaskCreatePinnedToCore(imuTask,    "imu",    4096, NULL, 4, &hImu,    1);
  xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, NULL, 3, &hSensor, 1);
  xTaskCreatePinnedToCore(navTask,    "nav",    4096, NULL, 2, &hNav,    1);
  xTaskCreatePinnedToCore(statusTask, "status", 4096, NULL, 1, &hStatus, 0);

  TimerHandle_t hb = xTimerCreate("heartbeat", pdMS_TO_TICKS(500), pdTRUE, NULL, heartbeatCb);
  xTimerStart(hb, 0);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}