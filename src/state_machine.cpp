#include "state_machine.h"
#include "locomotion.h"
#include "function.h"
#include "pid.h"
#include "button.h"
#include "display.h"
#include "line_sensor.h"
#include "mission.h"
#include "web_server.h"
#include "encoder_calibrate.h"

#define PID_TEST_DRIVE_MS 250u
#define MENU_ITEM_COUNT 6u

uint8_t current_state = STATE_IDLE;
bool pid_menu_active = false;
bool encoder_cal_menu_active = false;
uint8_t pid_menu_item = 0;
static bool is_calibrating = false;
static uint8_t menu_index = 0;

static const char *const kMenuLabels[MENU_ITEM_COUNT] = {
    "Start Mission", "Update Mission", "Calibrate Sensor", "Check Sensor",
    "Calibrate Encoder", "Tune PID"};

static const uint8_t kMenuTargetState[MENU_ITEM_COUNT] = {
    STATE_RUN_MISSION, STATE_WEB_SERVER, STATE_CALIBRATING,
    STATE_LINE_DEBUG,  STATE_CALIBRATE_ENCODER, STATE_PID_TUNING};

void runStateMachine() {
  const uint32_t now = millis();

  checkBatteryAlarm();

  if (!isPIDSettingsLoaded()) {
    initPIDController();
    loadPIDSettings();
  }

  if (!isCalibrationLoaded()) {
    loadCalibration();
    loadEncoderCalibration();
  }

  if (!isMissionLoaded()) {
    loadMissionFile();
  }

  enableHotspot();
  handleMissionWebServer();

  static uint8_t last_state = STATE_IDLE;
  
  if (justLoadedMission) {
    current_state = STATE_JUST_LOADED_MISSION;
    justLoadedMission = false;
  }

  bool btn3_poweroff = buttonHeld(BTN_3, 3000);

  if (isButtonDown(BTN_3) && current_state != STATE_PID_TUNING) {
    if (btn3_poweroff) {
      displayOLED("POWER OFF", "BTN3 > 3s", "Shutting down", "");
      power(false);
      return;
    } else {
      char countdown_buf[32];
      uint32_t held_ms = buttonHeldMs(BTN_3);
      uint32_t remain_ms = (held_ms < 3000) ? (3000 - held_ms) : 0;
      float remain_s = remain_ms / 1000.0f;
      snprintf(countdown_buf, sizeof(countdown_buf), "%.1f s", remain_s);
      const char *release_hint =
          (current_state == STATE_IDLE) ? "Release=Cancel" : "Release=Back";
      displayOLED("HOLD BTN3", "Power off in", countdown_buf, release_hint);
      return;
    }
  }

  if (current_state != STATE_IDLE && current_state != STATE_PID_TUNING &&
      isButtonReleased(BTN_3) && !btn3_poweroff) {
    current_state = STATE_IDLE;
    return;
  }

  if (current_state == STATE_IDLE) {
    if (isButtonPressed(BTN_2)) {
      menu_index = (menu_index + MENU_ITEM_COUNT - 1) % MENU_ITEM_COUNT;
      return;
    }
    if (isButtonPressed(BTN_1)) {
      menu_index = (menu_index + 1) % MENU_ITEM_COUNT;
      return;
    }
    if (isButtonPressed(BTN_4)) {
      current_state = kMenuTargetState[menu_index];
      return;
    }
  }

  const bool led_should_be_on = (current_state != STATE_IDLE);
  digitalWrite(LED_POWER_PIN, led_should_be_on ? HIGH : LOW);

  if (last_state != current_state) {
    if (current_state == STATE_PID_TUNING) {
      pid_menu_active = true;
      pid_menu_item = 0;
      stopMotors();
      resetPID();
    }
    if (current_state == STATE_IDLE) {
      pid_menu_active = false;
      pid_menu_item = 0;
      menu_index = 0;
      stopMotors();
    }

    if (current_state == STATE_RUN_MISSION) {
      delay(100);
    }

    if (last_state == STATE_CALIBRATING) {
      is_calibrating = false;
    }

    last_state = current_state;
  }

  switch (current_state) {

  case STATE_IDLE: {
    resetMissionState();
    disablePID();
    displayMenuOLED("SELECT MODE", kMenuLabels, MENU_ITEM_COUNT, menu_index);
    break;
  }

  case STATE_RUN_MISSION: {
    enablePID();
    if (last_state != STATE_RUN_MISSION) {
      delay(200);
    }
    runMission();
    break;
  }

  case STATE_CALIBRATING: {
      stopMotors();
      displayOLED("SWIPE LINE TO CALIBRATE", "BTN4=Finish", "BTN3=Back", "");

      if (!is_calibrating) {
          is_calibrating = true;
          lineSensorCalibrationBegin();  // Core-0 task now accumulates automatically
      }

      if (isButtonPressed(BTN_4)) {
          lineSensorCalibrationEnd();
          Serial.print("[LINE] CALIBRATION DONE. Thresholds: ");
          for (int i = 0; i < 16; i++) {
            Serial.printf("%d:%u ", i, getLineSensorThreshold(i));
          }
          Serial.println();

          is_calibrating = false;
          current_state = STATE_IDLE;
      }
      break;
  }

  case STATE_LINE_DEBUG: {
      static uint16_t rawSensor[16];
      static uint8_t digSensor[16];
      getLineSensorSnapshot(rawSensor, digSensor);

      char sensorMask[17];
      for (int i = 0; i < 16; i++) {
        sensorMask[i] = digSensor[i] ? '1' : '0';
      }
      sensorMask[16] = '\0';

      displayOLED("LINE DEBUG", sensorMask, "", "BTN3=BACK");

      for (int i = 0; i < 16; i++) {
        Serial.printf("raw %d: %u\n", i, rawSensor[i]);
      }
      break;
  }
  
  case STATE_PID_TUNING: {
    if (isButtonPressed(BTN_4)) {
      if (!pid_menu_active) {
        pid_menu_active = true;
        pid_menu_item = 0;
      } else if (pid_menu_item < 3) {
        pid_menu_item++;
      } else {
        current_state = STATE_IDLE;
      }
    }

    float kp_step = 0.01f;
    float ki_step = 0.001f;
    float kd_step = 0.001f;
    float base_step = 5.0f;

    auto pid_coefficient = getPIDCoefficient();
    float pid_current_Kp = pid_coefficient[0];
    float pid_current_Ki = pid_coefficient[1];
    float pid_current_Kd = pid_coefficient[2];
    float pid_base_speed = getPIDBaseSpeed();

    const bool btn1_fast =
      (isButtonDown(BTN_1)) &&
      (buttonHeldMs(BTN_1) >= 1000);

    const bool btn2_fast =
        (isButtonDown(BTN_2)) &&
        (buttonHeldMs(BTN_2) >= 1000);

    if (btn1_fast || btn2_fast) {
        kp_step = 0.10f;
        ki_step = 0.10f;
        kd_step = 0.01f;
        base_step = 20.0f;
    }

      static uint32_t last_repeat_ms = 0;
      const bool btn1_adjust =
          isButtonPressed(BTN_1) ||
          (btn1_fast && (now - last_repeat_ms >= 80));

      const bool btn2_adjust =
          isButtonPressed(BTN_2) ||
          (btn2_fast && (now - last_repeat_ms >= 80));

    if (pid_menu_active && btn1_adjust) {
      switch (pid_menu_item) {
      case 0:
        pid_current_Kp -= kp_step;
        if (pid_current_Kp < 0.0f) {
          pid_current_Kp = 0.0f;
        }
        break;
      case 1:
        pid_current_Ki -= ki_step;
        if (pid_current_Ki < 0.0f) {
          pid_current_Ki = 0.0f;
        }
        break;
      case 2:
        pid_current_Kd -= kd_step;
        if (pid_current_Kd < 0.0f) {
          pid_current_Kd = 0.0f;
        }
        break;
      case 3:
        pid_base_speed -= base_step;
        if (pid_base_speed < 0.0f) {
          pid_base_speed = 0.0f;
        }
        break;
      }
      savePIDSettings(pid_current_Kp, pid_current_Ki, pid_current_Kd, pid_base_speed);
      last_repeat_ms = now;
    }

    if (pid_menu_active && btn2_adjust) {
      switch (pid_menu_item) {
      case 0:
        pid_current_Kp += kp_step;
        if (pid_current_Kp > 50.0f) {
          pid_current_Kp = 50.0f;
        }
        break;
      case 1:
        pid_current_Ki += ki_step;
        if (pid_current_Ki > 10.0f) {
          pid_current_Ki = 10.0f;
        }
        break;
      case 2:
        pid_current_Kd += kd_step;
        if (pid_current_Kd > 50.0f) {
          pid_current_Kd = 50.0f;
        }
        break;
      case 3:
        pid_base_speed += base_step;
        if (pid_base_speed > 255.0f) {
          pid_base_speed = 255.0f;
        }
        break;
      }
      savePIDSettings(pid_current_Kp, pid_current_Ki, pid_current_Kd, pid_base_speed);
      last_repeat_ms = now;
    }

    static uint32_t pid_test_active_until_ms = 0;

    if (isButtonPressed(BTN_3)) {               
      pid_test_active_until_ms = now + PID_TEST_DRIVE_MS;             
      resetPID();       
    }

    const bool pid_test_active = (now < pid_test_active_until_ms);

    if (pid_test_active) {
      enablePID();
      followLinePID(false, false, pid_base_speed);
    } else if (isPIDEnabled()) {
      disablePID();
      stopMotors();
    }
    if (!pid_test_active) {
      char l2[32];
      char l3[32];
      char l4[32];

      if (pid_menu_active) {
        const char *selected = "KP";
        float value = pid_current_Kp;
        if (pid_menu_item == 1) {
          selected = "KI";
          value = pid_current_Ki;
        } else if (pid_menu_item == 2) {
          selected = "KD";
          value = pid_current_Kd;
        } else if (pid_menu_item == 3) {
          selected = "SPD";
          value = pid_base_speed;
        }

        snprintf(l2, sizeof(l2), "SEL:%s VAL:%.3f", selected, value);
        snprintf(l3, sizeof(l3), "BTN1=- BTN2=+");
        snprintf(l4, sizeof(l4), "BTN3=Test BTN4=Next");
        displayOLED("PID MENU", l2, l3, l4);
      } else {
        snprintf(l2, sizeof(l2), "KP:%.2f KI:%.3f", pid_current_Kp,
                 pid_current_Ki);
        snprintf(l3, sizeof(l3), "KD:%.2f SPD:%.0f", pid_current_Kd,
                 pid_base_speed);
        snprintf(l4, sizeof(l4), "BTN4=Menu");
        displayOLED("PID TUNING", l2, l3, l4);
      }
    }
    break;
  }

  case STATE_WEB_SERVER:
    printHotspotInformation();
    break;

  case STATE_JUST_LOADED_MISSION:
    break;

  case STATE_CALIBRATE_ENCODER: {
    static float current_val = 10.f;
    if (!encoder_cal_menu_active) {
      displayOLED("MENJALANKAN", "SAMPAI", "PULSA ENC", "250");
      calibrateEncoder();
      current_val = getCalibratedEncoderValue();
    }

    encoder_cal_menu_active = true;
    float val_step = 0.1f;

    const bool btn1_fast =
      (isButtonDown(BTN_1)) &&
      (buttonHeldMs(BTN_1) >= 1000);

    const bool btn2_fast =
        (isButtonDown(BTN_2)) &&
        (buttonHeldMs(BTN_2) >= 1000);

    if (btn1_fast || btn2_fast) {
      val_step = 1.f;
    }

    static uint32_t last_repeat_ms = 0;

    const bool btn1_adjust =
        isButtonPressed(BTN_1) ||
        (btn1_fast && (now - last_repeat_ms >= 80));

    const bool btn2_adjust =
        isButtonPressed(BTN_2) ||
        (btn2_fast && (now - last_repeat_ms >= 80));

    if (btn1_adjust) {
      current_val -= val_step;
      if (current_val < 1) current_val = 1;
    }

    if (btn2_adjust) {
      current_val += val_step;
      if (current_val > 1000) current_val = 1000;   
    }

    if (isButtonPressed(BTN_4) || isButtonPressed(BTN_3)) {
      displayOLED("ENCODER", "CALIBRATION", "SET", "");
      saveEncoderCalibration(current_val);
      encoder_cal_menu_active = false;
      delay(100);
      current_state = STATE_IDLE;
      return;
    }

    char line1[32];
    char line2[32];
    char line3[32];
    snprintf(line1, sizeof(line1), "SEJAUH (CM):%.2f", current_val);
    snprintf(line2, sizeof(line2), "BTN1=- BTN2=+");
    snprintf(line3, sizeof(line2), "BTN4=Done");
    displayOLED("ROBOT BERGERAK: ", line1, line2, line3);

    break;
  }
  default:
    displayOLED("ERROR", "Unknown state", "", "");
    break;
  }
}