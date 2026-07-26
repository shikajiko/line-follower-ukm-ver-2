# Dokumentasi Detail Program Pengetesan ESP32-S3

## 1. Tujuan Sistem
Program ini dibuat untuk mengetes seluruh modul hardware robot secara berurutan (sequential test) dengan kontrol tombol dan tampilan OLED.

Fitur utama:
- Sequence test dari sensor sampai aktuator.
- Navigasi manual antar state.
- Emergency power-off dengan long press tombol 3.
- Output debug muncul di OLED dan dimirror ke Serial.

## 2. Alur Inisialisasi
Urutan setup pada program utama:
1. INH1 diset LOW (motor disable saat boot)
2. initpower lalu power true
3. Inisialisasi buzzer (bunyi beep singkat)
4. Serial begin 115200 (USB CDC bila enable di build_flags)
5. initOLED
6. initButton
7. initEncoder
8. setupPCNT
9. initMotor
10. initADC
11. initMUX
12. initGY25
13. initServo

Setelah itu sistem menampilkan READY dan menunggu BUTTON1.

## 3. Kontrol Tombol
- BUTTON1 GPIO39
  - Dari IDLE: mulai sequence.
  - Saat test berjalan: next state.
- BUTTON2 GPIO40
  - Saat test berjalan: previous state.
- BUTTON3 GPIO21
  - Ditahan lebih dari 3 detik: power off sistem.
  - Saat ditahan, OLED menampilkan countdown.
  - Jika dilepas sebelum 3 detik: batal power off.
- BUTTON4 GPIO47
  - Reserved (belum dipakai di logika utama).

## 4. Daftar State Sequence
Urutan state di firmware:
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

## 5. Detail Per Sequence dan Output Debug

### 5.1 IDLE
Fungsi:
- Menunggu start test dari BUTTON1.

LCD:
- READY
- Press BTN1
- to start test

Serial mirror:
- [OLED]
- READY
- Press BTN1
- to start test
- ---

### 5.2 GY25
Fungsi:
- Membaca yaw/pitch/roll dari modul GY-25 via Serial1.
- Format data: frame biner 8-byte (0xAA ... 0x55), nilai int16 (x100 derajat).
- Timeout jika tidak ada frame valid selama 1000 ms.

LCD contoh:
- TEST: GY25
- Y:123.45 P:-01.23
- R:045.67 OK
- BTN1=Next BTN2=Prev

LCD saat timeout:
- TEST: GY25
- GY25: 123 deg
- TIMEOUT
- BTN1=Next

Serial mirror contoh:
- [OLED]
- TEST: GY25
- GY25: 123 deg
- OK
- BTN1=Next
- ---

### 5.3 BUTTON
Fungsi:
- Tahap validasi input tombol.

LCD:
- TEST: BUTTON
- B1:0 B2:1
- B3:1 B4:1
- Hold1=Next Hold2=Prev

Catatan:
- Pada state BUTTON, navigasi dibuat dengan HOLD (tahan) supaya tombol bisa ditap untuk dites tanpa langsung pindah state.

Serial mirror:
- [OLED]
- TEST: BUTTON
- Press BTN1/2/3/4
- to test
- BTN1=Next
- ---

### 5.4 BUZZER
Fungsi:
- Buzzer berkedip on/off selama durasi test.

Catatan:
- Di luar state BUZZER, buzzer dipaksa OFF (LOW).

LCD:
- TEST: BUZZER
- Buzzing...
- BTN1=Next

Serial mirror:
- [OLED]
- TEST: BUZZER
- Buzzing...
- BTN1=Next
- ---

### 5.5 ENC1
Fungsi:
- Menampilkan counter encoder 1.

LCD contoh:
- TEST: ENCODER 1
- ENC1: 245
- Rotate wheel L
- BTN1=Next

Serial mirror:
- [OLED]
- TEST: ENCODER 1
- ENC1: 245
- Rotate wheel L
- BTN1=Next
- ---

### 5.6 ENC2
Fungsi:
- Menampilkan counter encoder 2.

LCD contoh:
- TEST: ENCODER 2
- ENC2: 188
- Rotate wheel R
- BTN1=Next

Serial mirror:
- [OLED]
- TEST: ENCODER 2
- ENC2: 188
- Rotate wheel R
- BTN1=Next
- ---

### 5.7 SERVO1
Fungsi:
- Sweep servo 1 dari 1000 ke 2000 us.

LCD contoh:
- TEST: SERVO 1
- SRV1: 1500 us
- BTN1=Next

Serial mirror:
- [OLED]
- TEST: SERVO 1
- SRV1: 1500 us
- BTN1=Next
- ---

### 5.8 SERVO2
Fungsi:
- Sweep servo 2 dari 1000 ke 2000 us.

LCD contoh:
- TEST: SERVO 2
- SRV2: 1750 us
- BTN1=Next

Serial mirror:
- [OLED]
- TEST: SERVO 2
- SRV2: 1750 us
- BTN1=Next
- ---

### 5.9 MOTOR1
Fungsi:
- Forward-reverse ramp motor 1 dan tampilkan encoder 1.

LCD contoh:
- TEST: MOTOR 1
- M1 FWD PWM:120
- ENC1: 31
- BTN1=Next

Fase test:
- 0 sampai 2.5 detik: forward ramp 0 ke 255.
- 2.5 sampai 2.7 detik: BRAKE (PWM 0) untuk dead-time.
- 2.7 sampai 5.2 detik: reverse ramp 0 ke -255.

Catatan penting:
- Enable motor (INH1) dipertahankan beberapa ratus ms setelah command 0 untuk membantu transisi arah (sebagian driver butuh waktu enable saat input arah berubah).

Serial mirror:
- [OLED]
- TEST: MOTOR 1
- M1 FWD PWM:120
- ENC1: 31
- BTN1=Next
- ---

### 5.10 MOTOR2
Fungsi:
- Forward-reverse ramp motor 2 dan tampilkan encoder 2.

LCD contoh:
- TEST: MOTOR 2
- M2 REV PWM:80
- ENC2: 28
- BTN1=Next

Fase test:
- 0 sampai 2.5 detik: forward ramp 0 ke 255.
- 2.5 sampai 2.7 detik: BRAKE (PWM 0) untuk dead-time.
- 2.7 sampai 5.2 detik: reverse ramp 0 ke -255.

Catatan penting:
- Enable motor (INH1) dipertahankan beberapa ratus ms setelah command 0 untuk membantu transisi arah.

### 5.11 LED_POWER
Fungsi:
- LED refleksi untuk line sensor diuji dengan mode blink (kedip).

### 5.12 VBATT
Fungsi:
- Membaca tegangan pada pin VBAT_SENSE (ADC) dalam RAW (0-4095) dan mV.
- Menghitung estimasi tegangan baterai menggunakan faktor kalibrasi `VBAT_SCALE`:
  - `VBAT(mV) = SENSE(mV) * VBAT_SCALE`

LCD contoh:
- TEST: VBATT
- RAW:2152
- S:1808mV B:11.95V
- BTN1=Next BTN2=Prev

Cara kalibrasi:
- Ukur tegangan baterai aktual dengan multimeter (misal 11.95V).
- Baca `S:` di OLED (misal 1808mV).
- Hitung `VBAT_SCALE = 11950 / 1808 = 6.6106`.
- Set di build flags (platformio.ini): `-D VBAT_SCALE=6.6106f`.

Serial mirror:
- [OLED]
- TEST: MOTOR 2
- M2 REV PWM:80
- ENC2: 28
- BTN1=Next
- ---

### 5.11 LED_POWER
Fungsi:
- Uji LED pencahayaan line sensor (photodiode reflection LED).
- Memastikan LED_POWER_PIN aktif sebelum pembacaan line sensor.

LCD contoh:
- TEST: LED POWER
- LED ON for line
- sensor reflection
- BTN1=Next

Catatan transisi:
- LED_POWER_PIN berkedip terus-menerus (blink) selama state ini aktif.
- Tekan BUTTON1 untuk lanjut ke LINE_SENSOR.

Serial debug tambahan:
- [LED] STATE:10 PIN:16 OUT:ON/OFF (saat level output berubah)
- [LED_TEST] BLINK:ON/OFF BTN1=NEXT (periodik saat state LED_POWER)

### 5.12 LINE_SENSOR
Fungsi:
- LED_POWER_PIN tetap ON selama pembacaan.
- Baca 16 channel line sensor terus-menerus.
- Raw per 4 channel tetap berganti per 1 detik.
- Digital 16 channel divisualkan dalam 16 bar persegi di satu layar.

LCD contoh:
- RAW CH0-3
- 0:2110 1:1980
- 2:2050 3:1900
- (16 bar digital tampil di area bawah layar)

Serial debug khusus:
- [LINE] G0 RAW 0:2110 1:1980 2:2050 3:1900 DIG:1010011011000011

Catatan transisi:
- State LINE_SENSOR tidak auto timeout.
- Tekan BUTTON1 untuk lanjut ke DONE.

### 5.13 DONE
Fungsi:
- Menandakan test sequence selesai.

LCD:
- ALL TESTS DONE
- BTN1=Restart
- BTN2=Prev

Serial mirror:
- [OLED]
- ALL TESTS DONE
- BTN1=Restart
- BTN2=Prev
- ---

## 6. Emergency Power-Off BUTTON3
Saat BUTTON3 ditahan:

LCD contoh countdown:
- HOLD BTN3
- Power off in
- 2.3 s
- Release=Cancel

Serial mirror countdown:
- [OLED]
- HOLD BTN3
- Power off in
- 2.3 s
- Release=Cancel
- ---

Saat mencapai 3 detik:
- power false dipanggil.
- OLED menampilkan:
  - POWER OFF
  - BTN3 > 3s
  - Shutting down

## 7. Format Debug Serial
Serial menampilkan:
- Banner startup.
- Status inisialisasi.
- Mirror frame OLED dengan prefix [OLED].

Frame OLED ke serial dipublish saat:
- Frame pertama.
- Isi frame berubah, dengan throttle sekitar 200 ms agar tidak spam berlebih.

Khusus LINE_SENSOR:
- Menggunakan format [LINE] berkala sekitar 300 ms.
- Menampilkan grup raw aktif dan string digital 16 channel.

Khusus LED_POWER:
- Serial debug [LED] diprint saat output LED berubah.
- Serial debug [LED_TEST] diprint periodik untuk status blink.

## 8. Parameter Penting Runtime
- Debounce tombol: 50 ms.
- Hold BUTTON3 untuk power off: 3000 ms.
- Timeout data GY25: 1000 ms.
- Threshold line sensor digital: 2048.
- LED_POWER_PIN line illumination: GPIO16.
- Servo update: LEDC 50 Hz, resolusi 12-bit.

## 9. Pin Utama
- BUTTON1 39
- BUTTON2 40
- BUTTON3 21
- BUTTON4 47
- Encoder 41 dan 42
- Motor IN1 35, IN2 36, IN3 37, IN4 38, EN 2
- LED power line sensor 16
- Servo PWM1 6, PWM2 7
- MUX ADC 10, S0 13, S1 14, S2 12, S3 11
- GY25 RX 17, TX 18
- OLED I2C SDA 8, SCL 9, address 0x3C
- Power switch output 48

## 10. Ringkasan Operasional
1. Upload firmware.
2. Buka serial monitor 115200.
3. Pastikan tampilan READY.
4. Tekan BUTTON1 untuk mulai sequence.
5. Gunakan BUTTON2 untuk kembali state bila perlu.
6. Tahan BUTTON3 lebih dari 3 detik jika ingin mematikan sistem.

## 11. Cara Kerja Fungsi Secara Detail

### 11.1 Alur Fungsi Utama Program

Urutan eksekusi utama:
1. setup pada src/main.cpp memanggil seluruh inisialisasi perangkat.
2. loop pada src/main.cpp menjalankan pollButtons lalu runTestSequence secara periodik.
3. runTestSequence bertindak sebagai state machine untuk seluruh proses test.

Referensi kode:
- setup: [src/main.cpp](src/main.cpp#L5)
- loop: [src/main.cpp](src/main.cpp#L65)
- runTestSequence: [src/function.cpp](src/function.cpp#L1423)

### 11.2 Konfigurasi IO Perangkat

Semua mapping pin didefinisikan di [include/IO.h](include/IO.h).

Pin penting:
- Tombol: [include/IO.h](include/IO.h#L11), [include/IO.h](include/IO.h#L12), [include/IO.h](include/IO.h#L13), [include/IO.h](include/IO.h#L14)
- Motor: [include/IO.h](include/IO.h#L6), [include/IO.h](include/IO.h#L32), [include/IO.h](include/IO.h#L33), [include/IO.h](include/IO.h#L34), [include/IO.h](include/IO.h#L35)
- Encoder: [include/IO.h](include/IO.h#L9), [include/IO.h](include/IO.h#L8)
- Servo PWM: [include/IO.h](include/IO.h#L37), [include/IO.h](include/IO.h#L38)
- OLED I2C: [include/IO.h](include/IO.h#L54), [include/IO.h](include/IO.h#L55)
- MUX line sensor: [include/IO.h](include/IO.h#L57), [include/IO.h](include/IO.h#L58), [include/IO.h](include/IO.h#L59), [include/IO.h](include/IO.h#L60), [include/IO.h](include/IO.h#L61)
- GY25 UART: [include/IO.h](include/IO.h#L51), [include/IO.h](include/IO.h#L52)
- Power switch: [include/IO.h](include/IO.h#L63)

Fungsi inisialisasi IO:
- initpower: set pin power sebagai output di [src/function.cpp](src/function.cpp#L530)
- initBuzzer: set buzzer output di [src/function.cpp](src/function.cpp#L559)
- initButton: set semua tombol dengan `BUTTON_PINMODE` (default INPUT_PULLDOWN, ikut flag `BUTTON_ACTIVE_HIGH` di include/IO.h) di [src/function.cpp](src/function.cpp#L564)
- initButton: sekaligus inisialisasi LED_POWER_PIN sebagai output default LOW di [src/function.cpp](src/function.cpp#L564)
- initEncoder: set pin encoder input pullup di [src/function.cpp](src/function.cpp#L575)
- initMotor: set pin motor output dan kondisi awal low di [src/function.cpp](src/function.cpp#L580)
- initADC: set ADC 12-bit untuk line sensor di [src/function.cpp](src/function.cpp#L606)
- initMUX: set pin select mux output di [src/function.cpp](src/function.cpp#L613)
- initGY25: buka Serial1 115200 baud (parser frame 8-byte) di [src/function.cpp](src/function.cpp#L632)
- initServo: setup LEDC 50 Hz channel 0 dan 1 di [src/function.cpp](src/function.cpp#L647)
- setupPCNT: konfigurasi counter encoder unit 0 dan 1 di [src/function.cpp](src/function.cpp#L659)
- initOLED: start Wire dan SSD1306 di [src/function.cpp](src/function.cpp#L537)

### 11.3 Cara Mengambil Data

#### Data GY25
- Fungsi: readGY25Yaw di [src/function.cpp](src/function.cpp)
- Format frame yang dipakai firmware:
  - 8 byte: `[0]=0xAA, [1..2]=yaw, [3..4]=pitch, [5..6]=roll, [7]=0x55`
  - Nilai yaw/pitch/roll bertipe `int16_t` dengan skala x100 derajat.
- Alur detail pembacaan:
  1. Streaming parser: baca byte satu per satu dari `Serial1`.
  2. Sinkronisasi header: abaikan byte sampai menemukan `0xAA` saat index=0.
  3. Kumpulkan 8 byte ke buffer frame.
  4. Validasi tail: byte terakhir harus `0x55`.
  5. Decode yaw/pitch/roll dari big-endian (high byte dulu).
  6. Update `gy25_last_read = millis()` dan set `gy25_timeout = false`.
  7. Jika lebih dari 1000 ms tanpa frame valid, set `gy25_timeout = true`.

#### Data Encoder
- Fungsi baca: readEncoder di [src/function.cpp](src/function.cpp#L852)
- Fungsi reset: resetEncoder di [src/function.cpp](src/function.cpp#L866)
- Konfigurasi dasar PCNT: [src/function.cpp](src/function.cpp#L659)
  - Encoder 1 menggunakan `PCNT_UNIT_0` pada GPIO41.
  - Encoder 2 menggunakan `PCNT_UNIT_1` pada GPIO42.
  - `pos_mode = PCNT_COUNT_INC`, `neg_mode = PCNT_COUNT_DIS`.
  - Artinya hanya sisi pulsa tertentu yang menambah counter.
- Alur detail pembacaan:
  1. Pilih unit berdasarkan nomor encoder.
  2. Ambil nilai counter 16-bit melalui `pcnt_get_counter_value`.
  3. Simpan ke variabel global (`enc1_count` atau `enc2_count`).
  4. Kembalikan sebagai `int32_t` untuk konsistensi pemakaian di state test.
- Alur reset:
  1. Pause counter.
  2. Clear counter ke nol.
  3. Resume counter.
  4. Reset cache nilai global terkait encoder.

#### Data Line Sensor
- Pilih channel mux: selectMUXChannel di [src/function.cpp](src/function.cpp#L944)
- Baca raw: readLineSensor di [src/function.cpp](src/function.cpp#L1049)
- Baca digital threshold: readLineSensorDigital di [src/function.cpp](src/function.cpp#L1060)
- Alur detail akuisisi:
  1. Tentukan channel `0..15`.
  2. Set pin select MUX (`S0..S3`) berdasarkan bit channel.
  3. Delay settle sekitar 100 us agar sinyal stabil.
  4. Baca ADC 12-bit (`0..4095`) pada `MUX_ADC_PIN`.
  5. Simpan nilai raw ke array `line_sensor_raw[ch]`.
  6. Untuk status digital gunakan perbandingan:
     - `digital = raw > 2048 ? 1 : 0`
  7. Simpan hasil digital ke `line_sensor_digital[ch]`.
- Implementasi saat state LINE_SENSOR:
  - LED_POWER_PIN blink saat state LED_POWER, dan dipaksa HIGH saat state LINE_SENSOR.
  - Semua 16 channel dibaca terus-menerus tiap siklus.
  - Raw ditampilkan per grup 4 channel bergantian.
  - Digital 16 channel divisualkan sebagai 16 bar persegi.

### 11.4 Cara Melakukan Proses Kontrol

#### Kontrol Servo
- Fungsi: setServo di [src/function.cpp](src/function.cpp#L884)
- Setup PWM servo: [src/function.cpp](src/function.cpp#L647)
  - Frekuensi: 50 Hz (periode 20 ms)
  - Resolusi: 12-bit (duty 0..4095)
  - Channel LEDC: CH0 untuk servo1, CH1 untuk servo2
- Rumus konversi pulse ke duty:
  - `duty = (pulse_us * 4095) / 20000`
  - 1000 us kira-kira duty minimum kerja
  - 1500 us posisi tengah
  - 2000 us duty maksimum kerja
- Alur kontrol:
  1. Batasi duty maksimum 4095.
  2. Tulis duty ke channel terkait (`ledcWrite`).
  3. Simpan pulse terakhir ke variabel status servo.

#### Kontrol Motor
- Fungsi: setMotor di [src/function.cpp](src/function.cpp#L901)
- Catatan arsitektur kontrol saat ini:
  - Driver BTS7970B dikendalikan dengan PWM LEDC langsung pada pin IN1, IN2, IN3, IN4.
  - Pin `INH1_PIN` dipakai sebagai shared enable global.
  - Nilai `speed` langsung menentukan duty PWM (0 sampai 255) dan arah putaran.
- Alur detail:
  1. Clamp input `speed` ke rentang `-255..255`.
  2. Tentukan arah:
     - `speed > 0` maka forward.
     - `speed < 0` maka reverse.
  3. Hitung `pwm_val = abs(speed)` sebagai magnitude command.
  4. Tulis duty PWM ke channel LEDC motor:
     - Motor1 forward: IN1 = pwm_val, IN2 = 0
     - Motor1 reverse: IN1 = 0, IN2 = pwm_val
     - Motor2 forward: IN3 = pwm_val, IN4 = 0
     - Motor2 reverse: IN3 = 0, IN4 = pwm_val
  5. Simpan `motor1_speed` atau `motor2_speed` untuk telemetri.
  6. Aktifkan `INH1_PIN` jika salah satu motor speed tidak nol.
- Setup PWM motor (saat initMotor):
  - Frekuensi: 20 kHz
  - Resolusi: 8-bit (0..255)
  - Channel LEDC:
    - CH2 untuk IN1
    - CH3 untuk IN2
    - CH4 untuk IN3
    - CH5 untuk IN4
- Pada state test motor:
  - 0-2.5 s forward ramp 0->255
  - 2.5-5 s reverse ramp 0->-255
  - Encoder tetap dibaca realtime untuk melihat respons putaran.

#### Stop Motor
- Fungsi: stopMotors di [src/function.cpp](src/function.cpp#L931)
- Alur:
  1. Semua duty PWM channel motor diset 0.
  2. Pin enable `INH1_PIN` dipaksa LOW.
  3. Variabel speed internal diset nol.

#### Kontrol Power Sistem
- Fungsi inisialisasi: initpower di [src/function.cpp](src/function.cpp#L530)
- Fungsi kontrol: power di [src/function.cpp](src/function.cpp#L536)
- Alur:
  1. Pin `SW_POWER_PIN` diset sebagai output.
  2. `power(true)` memberi level HIGH untuk menyalakan jalur power switch.
  3. `power(false)` memberi level LOW untuk mematikan sistem.
  4. Dipakai pada startup dan emergency hold BUTTON3.

### 11.5 Proses Tombol dan Debounce

- Fungsi: pollButtons di [src/function.cpp](src/function.cpp#L1294)
- Mekanisme:
  1. Baca tombol.
  2. Simpan waktu perubahan state.
  3. Anggap valid jika stabil lebih dari debounce 50 ms.
  4. Set flag tombol ditekan untuk diproses state machine.

### 11.6 Proses State Machine Test

- Fungsi utama: runTestSequence di [src/function.cpp](src/function.cpp#L1423)

Urutan kerja runTestSequence:
1. Cek emergency hold BUTTON3 untuk power off.
2. Jika belum running, paksa state IDLE.
3. Proses tombol navigasi:
   - BUTTON1 untuk start atau next.
   - BUTTON2 untuk previous.
4. Eksekusi blok switch berdasarkan state aktif.
5. Setiap state punya proses, tampilan, dan kondisi transisi.

Implementasi proses spesifik:
- State motor forward-reverse ramp dengan encoder monitor:
  - MOTOR1 di [src/function.cpp](src/function.cpp#L1710)
  - MOTOR2 di [src/function.cpp](src/function.cpp#L1744)
- State LED power check sebelum pembacaan line sensor:
  - LED_POWER di [src/function.cpp](src/function.cpp#L1778)
- State line sensor continuous 16 channel:
  - LINE_SENSOR di [src/function.cpp](src/function.cpp#L1811)

### 11.7 Mekanisme Tampilan dan Debug

- Fungsi tampilan: displayOLED di [src/function.cpp](src/function.cpp#L697)
- Mekanisme:
  1. Menulis 4 baris text ke OLED.
  2. Membandingkan frame dengan frame sebelumnya.
  3. Jika berubah dan lolos throttle, frame dimirror ke Serial.
- Khusus line sensor, debug serial menggunakan format [LINE] agar ringkas.

### 11.8 Ringkas Logika Pengaturan Perangkat

Secara umum seluruh perangkat mengikuti pola yang sama:
1. Konfigurasi pin dan periferal saat setup.
2. Akuisisi data atau command output di loop test state.
3. Validasi dengan tampilan OLED dan serial debug.
4. Transisi state berdasarkan waktu dan tombol.

## 12. Penjelasan Program Lebih Detail (Dengan Kutipan Logika)

Bagian ini menjelaskan bagaimana program bekerja dari sisi implementasi internal, termasuk variabel status, keputusan if, dan transisi antar proses.

### 12.1 Variabel Global yang Menggerakkan Sistem

Variabel penting dibagi menjadi 5 kelompok:
1. Data sensor dan aktuator:
  - gy25_yaw, gy25_timeout
  - enc1_count, enc2_count
  - servo1_pulse, servo2_pulse
  - motor1_speed, motor2_speed
  - line_sensor_raw[16], line_sensor_digital[16]
2. Status tombol:
  - button1_pressed, button2_pressed
  - timer debounce tombol
3. Status state machine:
  - current_test_state
  - test_running
  - test_state_timer
  - test_phase
4. Safety power-off:
  - button3_hold_start
  - button3_poweroff_latched
5. Cache debug OLED:
  - prev1 sampai prev4 untuk mencegah spam serial

Efek praktis:
- Semua keputusan runtime diturunkan dari variabel ini.
- Jika perilaku test terasa aneh, titik cek pertama adalah nilai-nilai status tersebut.

### 12.2 Kutipan Logika Safety BUTTON3

Inti logika:
- Saat BUTTON3 ditekan terus, hitung lama tekan.
- Jika mencapai 3000 ms, panggil power false.
- Jika belum 3000 ms, tampilkan countdown di OLED.

Kutipan logika (disederhanakan):
- jika tombol 3 dalam kondisi ditekan (level BUTTON_PRESSED) dan hold_start masih nol maka set hold_start ke millis saat ini
- held_ms = millis sekarang dikurangi hold_start
- jika held_ms lebih besar sama dengan 3000 maka tampilkan POWER OFF lalu power false
- jika belum, tampilkan HOLD BTN3 dan sisa waktu
- jika tombol dilepas, reset hold_start dan latch

Tujuan desain:
- Menghindari power off tidak sengaja karena klik singkat.
- Tetap memberi feedback visual supaya operator tahu proses shutdown sedang dipicu.

### 12.3 Kutipan Logika Pengambilan Data GY25

Inti logika:
- Streaming parser frame 8 byte.
- Validasi header 0xAA dan tail 0x55.
- Parse yaw/pitch/roll masing-masing dari 2 byte (big-endian, x100 derajat).

Kutipan logika (disederhanakan):
- loop baca byte dari Serial1
- sinkronisasi: abaikan byte sampai dapat 0xAA saat index=0
- isi buffer sampai 8 byte
- validasi byte terakhir 0x55
- decode yaw/pitch/roll dari pasangan byte
- update gy25_last_read dan set timeout=false

Kenapa timeout diperlukan:
- Jika IMU lepas atau noise serial terjadi, sistem tidak boleh menampilkan data seolah valid terus.
- Timeout memberi indikator bahwa data saat ini stale.

### 12.4 Kutipan Logika PCNT Encoder

Konfigurasi awal:
- Set unit PCNT per encoder.
- Set mode hitung naik.
- Clear counter lalu resume.

Kutipan logika baca:
- jika encoderNum adalah 1, ambil counter dari unit 0
- jika encoderNum adalah 2, ambil counter dari unit 1
- kembalikan nilai sebagai int32

Kutipan logika reset:
- pause unit
- clear unit
- resume unit
- reset cache nilai global

Catatan:
- Implementasi ini single channel pulse counting.
- Cocok untuk monitoring kenaikan pulsa saat test putar roda.

### 12.5 Kutipan Logika PWM Servo

Setup penting:
- Frekuensi 50 Hz.
- Resolusi 12-bit.

Rumus utama yang digunakan:
- duty = pulse_us dikali 4095 dibagi 20000

Contoh interpretasi:
- 1000 us menghasilkan duty lebih kecil
- 1500 us posisi tengah
- 2000 us duty lebih besar

Kutipan logika:
- hitung duty dari pulse
- batasi duty maksimum 4095
- jika servo 1 tulis duty ke channel 0
- jika servo 2 tulis duty ke channel 1

### 12.6 Kutipan Logika Motor Forward-Reverse Ramp

Pola test per motor:
1. 0 sampai 2.5 detik forward ramp 0 ke 255
2. 2.5 sampai 5 detik reverse ramp 0 ke minus 255
3. stop motor lalu pindah state

Kutipan logika (disederhanakan):
- elapsed = millis sekarang dikurangi timer state
- jika elapsed kurang dari 2500, speed_cmd naik linear positif
- jika elapsed antara 2500 sampai 5000, speed_cmd turun linear negatif
- setMotor dengan speed_cmd
- baca encoder untuk feedback real time

Kutipan alur di setMotor (disederhanakan):
- clamp speed ke rentang -255 sampai 255
- pwm_val = absolut speed
- pilih channel IN maju atau IN mundur sesuai tanda speed
- tulis duty dengan ledcWrite ke channel terkait
- enable INH1 jika salah satu motor aktif

Kenapa ramp dipakai:
- Menguji respons driver dan mekanik saat percepatan bertahap.
- Memudahkan melihat transisi arah dan counter encoder di satu sesi test.

### 12.7 Kutipan Logika Line Sensor 16 Channel

Target tampilan:
- Raw tetap berganti per 4 channel setiap 1 detik.
- Digital 16 channel selalu terlihat sekaligus sebagai bar.

Kutipan logika:
- loop i dari 0 sampai 15: baca digital channel i
- tentukan disp_group dari selisih waktu per 1 detik
- tentukan ch_start = disp_group kali 4
- tampilkan raw ch_start sampai ch_start+3 di baris atas
- untuk i 0 sampai 15, gambar kotak digital
- jika digital channel i bernilai 1, isi kotak

Kelebihan pendekatan ini:
- Operator bisa melihat status digital semua channel dalam satu layar.
- Nilai raw tetap bisa dipantau bergilir untuk kalibrasi threshold.

### 12.8 Mekanisme Debug OLED ke Serial

Program tidak mencetak frame OLED mentah terus-menerus.
Ia memakai mekanisme ini:
1. Simpan frame sebelumnya (4 baris).
2. Bandingkan frame baru dengan frame lama.
3. Hanya kirim ke serial jika berubah dan lolos jeda minimum.

Kutipan logika:
- changed bernilai true jika salah satu baris berbeda
- time_ok bernilai true jika jeda lebih dari sekitar 200 ms
- jika first_frame atau changed dan time_ok maka print prefix OLED dan 4 baris

Manfaat:
- Debug serial tetap informatif.
- Mengurangi spam log yang membuat monitoring sulit.

### 12.9 Cara Melacak Bug Secara Praktis

Jika modul tidak bekerja, urutan diagnosis yang disarankan:
1. Cek apakah state masuk ke modul yang benar di OLED.
2. Cek serial mirror OLED dan format debug khusus modul.
3. Cek pin IO terkait pada daftar pin utama.
4. Cek nilai variabel status yang relevan di fungsi modul.
5. Cek transisi timer state apakah terlalu cepat atau tidak berpindah.

Pola ini mempersingkat debugging karena mengikuti alur program sebenarnya.
