// RoboField car firmware - Wemos D1 Mini (ESP8266)
// Same protocol/behaviour as the ESP32-C3 version; only chip-specific calls differ
// (WiFi event API, software PWM instead of LEDC, Servo library, EEPROM instead of Preferences).
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "protocol.h"
#include "config.h"
#include "secrets.h"

static WiFiUDP udp;
static Servo gripperServo;
static Adafruit_NeoPixel leds(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);
static WiFiEventHandler disconnectHandler;

static uint8_t robotId = 0;
static rf_command_t cmd = {};
static bool haveCmd = false;
static uint32_t lastCmdMs = 0;
static bool watchdogTripped = true;
static bool lowBattery = false;
static bool otaActive = false;
static uint16_t batteryMv = 0;
static float target[2] = {0, 0}, duty[2] = {0, 0};
static float desiredTarget[2] = {0, 0};
static float lastDir[2] = {0, 0};
static uint32_t reverseUntil[2] = {0, 0};

// ------------------------------------------------------------------ motors
static const uint32_t PWM_MAX = (1u << MOTOR_PWM_BITS) - 1;
static const float DEADZONE[2] = {LEFT_DEADZONE, RIGHT_DEADZONE};

static void writeMotor(int side, float d, bool brake) {
#ifdef DRIVER_DUAL_PWM
  int in1 = side == 0 ? PIN_L_IN1 : PIN_R_IN1;
  int in2 = side == 0 ? PIN_L_IN2 : PIN_R_IN2;
  if (brake) { analogWrite(in1, PWM_MAX); analogWrite(in2, PWM_MAX); return; }
  float mag = fabsf(d);
  float comp = mag > 0.001f ? min(1.0f, DEADZONE[side] + (1.0f - DEADZONE[side]) * mag) : 0.0f;
  uint32_t v = (uint32_t)(comp * PWM_MAX);
  if (d > 0) { analogWrite(in1, v); analogWrite(in2, 0); }
  else       { analogWrite(in1, 0); analogWrite(in2, v); }
#else
  int pwm = side == 0 ? PIN_PWMA : PIN_PWMB;
  int in1 = side == 0 ? PIN_AIN1 : PIN_BIN1;
  int in2 = side == 0 ? PIN_AIN2 : PIN_BIN2;
  if (brake) { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); analogWrite(pwm, 0); return; }
  float mag = fabsf(d);
  float comp = mag > 0.001f ? min(1.0f, DEADZONE[side] + (1.0f - DEADZONE[side]) * mag) : 0.0f;
  digitalWrite(in1, d > 0); digitalWrite(in2, d < 0);
  analogWrite(pwm, (uint32_t)(comp * PWM_MAX));
#endif
}

static void motorsOff() {
  desiredTarget[0] = desiredTarget[1] = 0;
  target[0] = target[1] = duty[0] = duty[1] = 0;
  lastDir[0] = lastDir[1] = 0;
  reverseUntil[0] = reverseUntil[1] = 0;
  writeMotor(0, 0, false); writeMotor(1, 0, false);
#ifdef DRIVER_TB6612
  digitalWrite(PIN_STBY, LOW);
#endif
}

static void motorsSetup() {
  analogWriteRange(PWM_MAX);
  analogWriteFreq(MOTOR_PWM_HZ);
#ifdef DRIVER_DUAL_PWM
  for (int p : {PIN_L_IN1, PIN_L_IN2, PIN_R_IN1, PIN_R_IN2}) pinMode(p, OUTPUT);
#else
  for (int p : {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2, PIN_STBY}) pinMode(p, OUTPUT);
#endif
  motorsOff();
}

static void motorsUpdate(float dt, uint32_t now, bool brake) {
  (void)now;
  target[0] = desiredTarget[0];
  target[1] = desiredTarget[1];
  float step = DUTY_RAMP_PER_S * dt;
  for (int i = 0; i < 2; i++) {
    float e = target[i] - duty[i];
    duty[i] += e > step ? step : (e < -step ? -step : e);
  }
#ifdef DRIVER_TB6612
  digitalWrite(PIN_STBY, (duty[0] != 0 || duty[1] != 0 || brake) ? HIGH : LOW);
#endif
  writeMotor(0, LEFT_REVERSED ? -duty[0] : duty[0], brake);
  writeMotor(1, RIGHT_REVERSED ? -duty[1] : duty[1], brake);
}

// ------------------------------------------------------------------ servo, leds, battery
static void setServo(uint8_t angle) {
  gripperServo.write(constrain(angle, 0, 180));
}

static void setLeds(const uint8_t *rgb) {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]);
  leds.show();
}

static void setAllLeds(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, r, g, b);
  leds.show();
}

static void updateBattery() {
#if ENABLE_BATTERY_MONITOR
  batteryMv = (uint16_t)((analogRead(PIN_VBAT) / 1023.0f) * 3200.0f * VBAT_DIVIDER);
  if (batteryMv < VBAT_CUTOFF_MV && batteryMv > 1000) lowBattery = true;
  else if (batteryMv > VBAT_RECOVER_MV) lowBattery = false;
#else
  batteryMv = 0;
  lowBattery = false;
#endif
}

// ------------------------------------------------------------------ identity (EEPROM, not Preferences)
static void setRobotId(uint8_t id) {
  if (id > 19) return;
  robotId = id;
  EEPROM.write(0, id);
  EEPROM.commit();
  Serial.printf("robot id set to %d (restart to apply the new OTA name)\n", id);
}

static void handleSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line.trim();
      if (line.startsWith("id ")) setRobotId(line.substring(3).toInt());
      else if (line == "info") Serial.printf("id %d  ip %s  rssi %d  battery %u mV  fw %d\n", robotId,
                                             WiFi.localIP().toString().c_str(), WiFi.RSSI(), batteryMv, FW_VERSION);
      line = "";
    } else line += c;
  }
}

// ------------------------------------------------------------------ network
static void handleUdp() {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    uint8_t buf[32];
    int len = udp.read(buf, sizeof(buf));
    if (len == sizeof(rf_command_t)) {
      rf_command_t *c = (rf_command_t *)buf;
      if (c->magic != RF_MAGIC || c->version != RF_VERSION || c->type != RF_TYPE_COMMAND) continue;
      if (rf_crc8(buf, len - 1) != c->crc) continue;
      if (c->robot_id != robotId && c->robot_id != RF_BROADCAST_ID) continue;
      cmd = *c; haveCmd = true;
    } else if (len == sizeof(rf_config_t)) {
      rf_config_t *c = (rf_config_t *)buf;
      if (c->magic == RF_MAGIC && c->type == RF_TYPE_CONFIG && rf_crc8(buf, len - 1) == c->crc) setRobotId(c->new_id);
    }
  }
}

static void sendStatus() {
  rf_status_t s = {RF_MAGIC, RF_VERSION, RF_TYPE_STATUS, robotId, cmd.seq, batteryMv, FW_VERSION,
                   (uint8_t)((watchdogTripped ? RF_STATUS_WATCHDOG : 0) | (lowBattery ? RF_STATUS_LOW_BATTERY : 0)),
                   (int8_t)WiFi.RSSI(), 0};
  s.crc = rf_crc8((uint8_t *)&s, sizeof(s) - 1);
  udp.beginPacket(IPAddress(255, 255, 255, 255), RF_UDP_STATUS_PORT);
  udp.write((uint8_t *)&s, sizeof(s));
  udp.endPacket();
}

static void setupOta() {
  char host[32];
  snprintf(host, sizeof(host), "robofield-car-%d", robotId);
  ArduinoOTA.setHostname(host);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { otaActive = true; motorsOff(); setAllLeds(60, 0, 60); });
  ArduinoOTA.onEnd([]() { setAllLeds(0, 60, 0); });
  ArduinoOTA.onError([](ota_error_t e) { otaActive = false; setAllLeds(60, 0, 0); });
  ArduinoOTA.begin();
  Serial.printf("OTA ready as %s.local\n", host);
}

// ------------------------------------------------------------------ main
void setup() {
  Serial.begin(115200);
  motorsSetup();
  gripperServo.attach(PIN_SERVO);
  setServo(90);
  leds.begin(); leds.setBrightness(LED_BRIGHTNESS); setAllLeds(0, 0, 40);

  EEPROM.begin(4);
  robotId = EEPROM.read(0);
  if (robotId > 19) robotId = 0;        // unwritten flash reads as 0xFF

  WiFi.mode(WIFI_STA);
  disconnectHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected &evt) {
    Serial.printf("disconnect reason: %d\n", (int)evt.reason);
  });
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setOutputPower(8.5);
  WiFi.disconnect(true);
  delay(300);

  Serial.println("scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  for (int i = 0; i < n; i++) {
    Serial.printf("  [%d] \"%s\"  rssi %d  ch %d  auth %d\n", i, WiFi.SSID(i).c_str(), WiFi.RSSI(i),
                  WiFi.channel(i), (int)WiFi.encryptionType(i));
  }

  for (int attempt = 1; attempt <= 5 && WiFi.status() != WL_CONNECTED; attempt++) {
    Serial.printf("connect attempt %d...\n", attempt);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
      setAllLeds(0, 0, (millis() / 250) % 2 ? 40 : 0); delay(300); handleSerial();
      Serial.printf("wifi status: %d\n", WiFi.status());
    }
    if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(true); delay(1500); }
  }

  udp.begin(RF_UDP_CMD_PORT);
  setupOta();
  Serial.printf("RoboField car %d  ip %s  fw %d\n", robotId, WiFi.localIP().toString().c_str(), FW_VERSION);
}

void loop() {
  static uint32_t lastUs = micros(), lastStatus = 0, lastBlink = 0;
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f; lastUs = nowUs;
  uint32_t now = millis();

  ArduinoOTA.handle();
  handleSerial();
  if (otaActive) return;
  if (WiFi.status() != WL_CONNECTED) {
    motorsOff(); watchdogTripped = true;
    setAllLeds(0, 0, (now / 250) % 2 ? 40 : 0);
    return;
  }

  handleUdp();
  bool brake = false;
  if (haveCmd) {
    haveCmd = false; lastCmdMs = now; watchdogTripped = false;
    bool enable = (cmd.flags & RF_FLAG_ENABLE) && !lowBattery;
    brake = cmd.flags & RF_FLAG_BRAKE;
    desiredTarget[0] = enable ? constrain(cmd.left, -1000, 1000) / 1000.0f : 0;
    desiredTarget[1] = enable ? constrain(cmd.right, -1000, 1000) / 1000.0f : 0;
    setServo(cmd.servo);
    if (!lowBattery) setLeds(cmd.leds);
  }
  if (!watchdogTripped && now - lastCmdMs > WATCHDOG_MS) {
    watchdogTripped = true; motorsOff();
  }
  motorsUpdate(dt, now, brake && !watchdogTripped);

  if (now - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = now; updateBattery();
    if (lowBattery) motorsOff();
    sendStatus();
  }
  if (lowBattery && now - lastBlink > 300) {
    lastBlink = now; static bool on = false; on = !on; setAllLeds(on ? 80 : 0, 0, 0);
  } else if (watchdogTripped && !lowBattery && now - lastBlink > 1000) {
    lastBlink = now; setAllLeds(0, 15, 0);
  }
}