#include "IO.h"
#include "function.h"
#include <Arduino.h>

void setup() {
  initpower();
  power(false); // Assert power enable ASAP (avoid floating enable during boot)
  pinMode(INH1_PIN, OUTPUT);
  digitalWrite(INH1_PIN, LOW); // Ensure power is off during setup
  delay(3000); // Wait for power to stabilize before initializing peripherals
  power(true); // Assert power enable ASAP (avoid floating enable during boot)
  initBuzzer();
  digitalWrite(BUZZER_PIN, HIGH);
  delay(500);
  digitalWrite(BUZZER_PIN, LOW);

  delay(450); // keep ~1s total settle time like before

  Serial.begin(115200);
// #if ARDUINO_USB_CDC_ON_BOOT
//   // Avoid long blocking writes when USB CDC is not connected.
//   Serial.setTxTimeoutMs(0);
// #endif
//   // If using native USB CDC, the host may need a moment to enumerate/open the
//   // port. Waiting avoids missing the first log lines.
// #if ARDUINO_USB_CDC_ON_BOOT
//   {
//     const unsigned long start = millis();
//     while (!Serial && (millis() - start) < 2000) {
//       delay(10);
//     }
//   }
// #endif
  delay(1000);

  Serial.println("\n\n=== ESP32-S3 HARDWARE TEST FRAMEWORK ===");
  Serial.println("Initializing all peripherals...");

  // Initialize display first
  initOLED();
  delay(500);

  // Initialize all peripherals
  initButton();
  initEncoder();
  setupPCNT();
  initMotor();
  initADC();
  initMUX();
  initGY25();
  initServo();

  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(100);
  }

  displayOLED("READY", "Press BTN1", "to start test", "");
  Serial.println("All peripherals initialized!");
  Serial.println("Press BUTTON1 to start tests.");
}

void loop() {
  // Poll button inputs
  pollButtons();

  // Run test sequence state machine
  // runTestSequence();

  // Small delay to prevent overwhelming the CPU
  delay(50);
}