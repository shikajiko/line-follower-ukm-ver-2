#include "IO.h"
#include "function.h"
#include "button.h"
#include "line_sensor.h"
#include "state_machine.h"
#include "display.h"
// #include "locomotion.h"
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
  readLineSensors();
  // Run test sequence state machine
  // runTestSequence();
  runStateMachine();
  // Small delay to prevent overwhelming the CPU
  delay(50);
}