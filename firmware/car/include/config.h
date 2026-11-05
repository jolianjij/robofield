// RoboField car configuration. Pins match docs/field.md / the wiring guide.
#pragma once

// ---------------- motor driver ----------------
// DRIVER_DUAL_PWM : 2 inputs per motor (MX1508, DRV8833, L9110S) - PWM on IN1 or IN2
// DRIVER_TB6612   : PWM + 2 direction pins per motor + STBY
#define DRIVER_DUAL_PWM
// #define DRIVER_TB6612

#ifdef DRIVER_DUAL_PWM
  #define PIN_L_IN1 3
  #define PIN_L_IN2 4
  #define PIN_R_IN1 6
  #define PIN_R_IN2 7
#else
  #define PIN_PWMA 1
  #define PIN_AIN1 3
  #define PIN_AIN2 4
  #define PIN_PWMB 5
  #define PIN_BIN1 6
  #define PIN_BIN2 7
  #define PIN_STBY 21
#endif

#define PIN_SERVO 20
#define PIN_LEDS 10
#define PIN_VBAT 0
#define NUM_LEDS 3

// flip a motor in software instead of swapping its wires
#define LEFT_REVERSED  false
#define RIGHT_REVERSED false

// ---------------- behaviour ----------------
#define MOTOR_PWM_HZ 20000
#define MOTOR_PWM_BITS 10            // duty 0..1023
#define SERVO_HZ 50
#define SERVO_BITS 14
#define WATCHDOG_MS 200              // no command for this long -> motors off
#define STATUS_PERIOD_MS 100         // status broadcast, 10 Hz
#define DUTY_RAMP_PER_S 6.0f         // max duty change per second (= 2000 mm/s^2): stops wheelies
// battery: 2S pack (7.4 V nominal, 8.4 V full) on GPIO0 through 100k (top) / 33k (bottom)
#define VBAT_DIVIDER 4.03f           // (100k + 33k) / 33k
#define VBAT_CUTOFF_MV 6600          // 3.3 V per cell: motors off below this
#define VBAT_RECOVER_MV 7000         // back on above this
#define LED_BRIGHTNESS 80
#define FW_VERSION 2
#define OTA_PASSWORD "robofield"     // change this, and in platformio.ini upload_flags
