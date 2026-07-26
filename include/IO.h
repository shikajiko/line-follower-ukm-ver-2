#ifndef IO_H
#define IO_H

// pin definitions
#define VBAT_SENSE_PIN 1
#define INH1_PIN 2

#define ENC2_A_PIN 41
#define ENC1_A_PIN 42

#define BUTTON1_PIN 39
#define BUTTON2_PIN 40
#define BUTTON3_PIN 21
#define BUTTON4_PIN 47

// ===== Button wiring polarity (SATU flag untuk semua logika tombol) =====
// Ganti nilai BUTTON_ACTIVE_HIGH ini saja untuk troubleshooting:
//   true  = tombol active-HIGH  -> idle LOW,  ditekan HIGH (pull-DOWN)
//   false = tombol active-LOW   -> idle HIGH, ditekan LOW  (pull-UP)
#define BUTTON_ACTIVE_HIGH true

#if BUTTON_ACTIVE_HIGH
#define BUTTON_PRESSED HIGH        // level saat tombol ditekan
#define BUTTON_RELEASED LOW        // level saat tombol lepas (idle)
#define BUTTON_PINMODE INPUT_PULLDOWN
#else
#define BUTTON_PRESSED LOW
#define BUTTON_RELEASED HIGH
#define BUTTON_PINMODE INPUT_PULLUP
#endif

#define IN1_PIN 35
#define IN2_PIN 36
#define IN3_PIN 37
#define IN4_PIN 38

#define PWM1_PIN 7
#define PWM2_PIN 6

#define TXD2_PIN 4
#define RXD2_PIN 5
#define TXD0_PIN 43
#define RXD0_PIN 44

#define BUZZER_PIN 15
#define LED_POWER_PIN 16

// GY-25 UART wiring (based on proven working sketch):
// - ESP32-S3 RX pin connects to sensor TX
// - ESP32-S3 TX pin connects to sensor RX
#define GY25_RX_PIN 18
#define GY25_TX_PIN 17

#define SDA_PIN 8
#define SCL_PIN 9

#define MUX_ADC_PIN 10
#define MUX_S0_PIN 13
#define MUX_S1_PIN 14
#define MUX_S2_PIN 12
#define MUX_S3_PIN 11

#define SW_POWER_PIN 48
#define GPIO0_PIN 0

#endif