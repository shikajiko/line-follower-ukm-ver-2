# ESP32-S3 Hardware Test Framework - Build Summary

## Build Status

- Last verified compile: SUCCESS
- Framework: Arduino on Espressif32
- Board: esp32s3usbotg

## Binary Files Generated

- bootloader.bin around 15 KB
- firmware.bin around 317 KB
- partitions.bin around 3 KB

## Memory Usage (Typical)

- RAM around 5.9 percent
- Flash around 9.5 percent

## Hardware Modules Integrated

1. GY25 IMU on Serial1, 115200 baud, RX GPIO18 and TX GPIO17 (frame biner 8-byte).
2. Dual encoder counter using PCNT on GPIO41 and GPIO42.
3. Servo control on PWM1 GPIO6 and PWM2 GPIO7 using LEDC 50 Hz.
4. Motor control for BTS7970B on IN1-IN4 with shared enable INH1 GPIO2, using LEDC PWM 20 kHz 8-bit.
5. Line sensor 16 channel via MUX select GPIO13, GPIO14, GPIO12, GPIO11 and ADC GPIO10.
6. Button control on GPIO39, GPIO40, GPIO21, GPIO47. Polaritas diatur satu flag `BUTTON_ACTIVE_HIGH` di include/IO.h (default true = active-HIGH / INPUT_PULLDOWN).
7. Buzzer output on GPIO15.
8. OLED SSD1306 128x64 at I2C address 0x3C.
9. Main power switch control on GPIO48 via initpower and power function.
10. VBATT sense on GPIO1 (ADC) dengan faktor kalibrasi `VBAT_SCALE`.

## Test Sequence States in Code

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

Catatan transisi terbaru:
- MOTOR1 dan MOTOR2 masing-masing berjalan 5 detik dengan pola forward-reverse ramp.
- LED_POWER berkedip terus (blink) sebagai verifikasi lampu refleksi line sensor sampai BUTTON1 ditekan.
- LINE_SENSOR berjalan terus-menerus (continuous) sampai user menekan BUTTON1 untuk lanjut ke DONE.

## Global Safety Action

- Hold BUTTON3 for more than 3 seconds to power off system.
- OLED shows countdown while BUTTON3 is held.
- Releasing BUTTON3 before 3 seconds cancels shutdown.

## OLED and I2C Notes

- OLED initialized with SSD1306_SWITCHCAPVCC and address 0x3C.
- I2C bus is explicitly started using SDA_PIN and SCL_PIN from IO definitions.

## Serial Output Notes

- Startup prints framework banner and initialization status.
- OLED initialization success is printed to Serial.
- Sebagian besar state menampilkan mirror frame OLED dengan prefix [OLED].
- State LED_POWER menambahkan debug [LED] (saat status berubah) dan [LED_TEST] (status blink periodik).
- State LINE_SENSOR memakai debug serial khusus format [LINE] agar ringkas untuk 16 channel.

## Motor Test Behavior

- MOTOR1:
	- 0 sampai 2.5 detik forward ramp 0 ke 255.
	- 2.5 sampai 2.7 detik brake/dead-time (PWM 0).
	- 2.7 sampai 5.2 detik reverse ramp 0 ke -255.
	- PWM motor ditulis real-time via LEDC (duty 0..255, frekuensi 20 kHz).
	- Encoder 1 tetap ditampilkan selama test.
- MOTOR2:
	- 0 sampai 2.5 detik forward ramp 0 ke 255.
	- 2.5 sampai 2.7 detik brake/dead-time (PWM 0).
	- 2.7 sampai 5.2 detik reverse ramp 0 ke -255.
	- PWM motor ditulis real-time via LEDC (duty 0..255, frekuensi 20 kHz).
	- Encoder 2 tetap ditampilkan selama test.

## Line Sensor Test Behavior

- LED_POWER_PIN dipaksa ON selama state LED_POWER dan LINE_SENSOR.
- Semua 16 channel dibaca terus-menerus.
- OLED menampilkan:
	- Raw value per 4 channel bergantian per 1 detik.
	- Visual digital 16 bar persegi dalam satu layar (0 kosong, 1 terisi).
- Serial menampilkan baris [LINE] berisi grup raw aktif dan string digital 16 bit.

## Files

- include/IO.h for pin map
- include/function.h for module API
- src/function.cpp for core hardware test logic
- src/main.cpp for setup and loop
- platformio.ini for build configuration

## Upload

- Run PlatformIO upload target after selecting the correct COM port.

## Known Limits

- GY25 timeout uses 1 second no-data threshold.
- Line sensor digital state uses threshold 2048.
- Servo duty conversion uses 12-bit duty over 20 ms period.
