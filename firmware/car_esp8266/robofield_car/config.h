// RoboField car configuration - Wemos D1 Mini (ESP8266)
#pragma once

// ---------------- motor driver ----------------
#define DRIVER_DUAL_PWM
// #define DRIVER_TB6612

#ifdef DRIVER_DUAL_PWM
  #define PIN_R_IN1 D5
  #define PIN_R_IN2 D6
  #define PIN_L_IN1 D7
  #define PIN_L_IN2 D3
#else
  #define PIN_PWMA D5
  #define PIN_AIN1 D6
  #define PIN_AIN2 D1
  #define PIN_PWMB D7
  #define PIN_BIN1 D8
  #define PIN_BIN2 D0
  #define PIN_STBY D3
#endif

#define PIN_SERVO D2
#define PIN_LEDS D1     // moved here since D7 is now a motor pin
#define PIN_VBAT A0
#define NUM_LEDS 3
#define LEFT_DEADZONE  0.26f
#define RIGHT_DEADZONE 0.20f
#define LEFT_REVERSED  false
#define RIGHT_REVERSED false
#define LEFT_TRIM  1.0f
#define RIGHT_TRIM 1.0f   // lower whichever wheel is "winning" the drift
// ---------------- behaviour ----------------
#define MOTOR_PWM_HZ 1000
#define MOTOR_PWM_BITS 10
#define WATCHDOG_MS 200
#define STATUS_PERIOD_MS 100
#define DUTY_RAMP_PER_S 6.0f
#define REVERSE_DEAD_MS 80

#define ENABLE_BATTERY_MONITOR false
#define VBAT_DIVIDER 4.03f
#define VBAT_CUTOFF_MV 6600
#define VBAT_RECOVER_MV 7000
#define LED_BRIGHTNESS 80
#define FW_VERSION 5
#define OTA_PASSWORD "robofield"