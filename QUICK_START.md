# Quick Start - ESP32-S3 Hardware Test Framework

## 1. Build and Upload

- Build: pio run
- Upload: pio run --target upload

## 2. Open Serial Monitor

- Port: choose detected COM port
- Baud rate: 115200
- Typical startup output:
  - ESP32-S3 HARDWARE TEST FRAMEWORK
  - Initializing all peripherals
  - OLED Initialized successfully
  - All peripherals initialized
  - Press BUTTON1 to start tests

## 3. Button Controls

- BUTTON1 GPIO39: Start test and next state.
- BUTTON2 GPIO40: Previous state.
- BUTTON3 GPIO21: Hold longer than 3 seconds to power off.
- BUTTON4 GPIO47: Reserved.

## 4. BUTTON3 Hold to Power Off

- While BUTTON3 is held, OLED shows shutdown countdown.
- Release before 3 seconds to cancel.
- At 3 seconds system calls power false.

## 5. Sequence Flow

1. IDLE
2. GY25
3. BUTTON
4. BUZZER
5. ENC1
6. ENC2
7. SERVO1
8. SERVO2
9. MOTOR1
10. MOTOR2
11. LED_POWER
12. VBATT
13. LINE_SENSOR
14. DONE

Catatan:
- Dari LINE_SENSOR, tekan BUTTON1 untuk lanjut ke DONE.
- State LED_POWER berkedip terus sampai BUTTON1 ditekan untuk masuk LINE_SENSOR.

## 6. Key Pin Map

- Encoder: GPIO41 ENC1 and GPIO42 ENC2.
- Servo: GPIO6 PWM1 and GPIO7 PWM2.
- Motor: GPIO35, GPIO36, GPIO37, GPIO38 and enable GPIO2.
  - Motor PWM: LEDC 20 kHz, 8-bit duty on IN1-IN4.
- Line sensor MUX select: GPIO13, GPIO14, GPIO12, GPIO11.
- Line sensor ADC: GPIO10.
- VBAT sense: GPIO1 (ADC) untuk VBAT_SENSE_PIN.
- GY25 UART: RX GPIO18 dan TX GPIO17 pada 115200 baud (frame biner 8-byte).
- Buttons: GPIO39, GPIO40, GPIO21, GPIO47.
- Buzzer: GPIO15.
- LED power line sensor: GPIO16.
- OLED I2C: SDA GPIO8 and SCL GPIO9, address 0x3C.
- Power switch output: GPIO48.

## 7. Troubleshooting

- OLED blank: check 0x3C address and SDA/SCL wiring.
- Encoder count does not move: check GPIO41 and GPIO42 wiring.
- Motor not moving / tidak mau reverse: check INH1 enable, wiring IN1-IN4, dan pastikan polaritas motor + suplai driver benar.
- GY25 timeout: verify UART wiring dan baud 115200.
- VBATT tidak sesuai multimeter: kalibrasi `VBAT_SCALE`.
- Line sensor static value: verify MUX select pins and ADC pin.

## 8. Runtime Test Pattern

- MOTOR1 dan MOTOR2:
  - 0 sampai 2.5 detik forward ramp.
  - 2.5 sampai 2.7 detik brake/dead-time (PWM 0).
  - 2.7 sampai 5.2 detik reverse ramp.
  - Duty PWM motor berubah real-time mengikuti ramp (LEDC 20 kHz, 0..255).
  - Encoder tetap tampil selama motor test.

- VBATT:
  - Menampilkan RAW ADC, S (sense mV), dan B (estimasi tegangan baterai).
  - Rumus: `B(mV) = S(mV) * VBAT_SCALE`.
  - Set kalibrasi di platformio.ini, contoh: `-D VBAT_SCALE=6.6106f`.
- LINE_SENSOR:
  - LED_POWER_PIN menyala selama pembacaan untuk bantu pantulan cahaya photodiode.
  - Tetap di satu state secara continuous.
  - Raw per 4 channel berganti tiap 1 detik.
  - Digital 16 channel divisualkan dalam 16 bar persegi pada satu layar.

- LED_POWER:
  - LED_POWER_PIN berkedip ON/OFF terus-menerus.
  - Tekan BUTTON1 untuk lanjut ke LINE_SENSOR.
  - Serial debug menampilkan [LED] dan [LED_TEST].

## 9. Important Runtime Values

- Button debounce: 50 ms
- Power-off hold time on BUTTON3: 3000 ms
- GY25 timeout: 1000 ms
- Line sensor threshold: 2048
- Servo pulse range used in test: 1000 to 2000 microseconds

Status: firmware compiled and validated.
