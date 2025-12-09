#include <Arduino.h>
#include <Bluepad32.h>
#include <DFPlayerMini_Fast.h>
#include <EEPROM.h>
#include <ESP32Servo.h>
#include <driver/mcpwm.h>
#include <esp_log.h>

#ifdef DISABLE_BROWNOUT_DETECTOR
#include <driver/rtc_io.h>
#include <soc/rtc_cntl_reg.h>
#endif

static auto MAIN_TAG = "RC_TANK";

// 핀 정의
#define DFPLAYER_RX 32
#define DFPLAYER_TX 33
#define LEFT_TRACK_IN1 25
#define LEFT_TRACK_IN2 26
#define RIGHT_TRACK_IN1 27
#define RIGHT_TRACK_IN2 13
#define TURRET_IN1 22
#define TURRET_IN2 21
#define CANNON_LED_PIN 4
#define HEADLIGHT_PIN 16
#define CANNON_MOUNT_SERVO_PIN 19  // 포 마운트 서보 모터 핀 (우측 Y축으로 각도 조절)
#define CANNON_SERVO_PIN 18        // 포신 서보 모터 핀 (A 버튼으로 당기기)

// MCPWM 설정 (모든 모터는 MCPWM_UNIT_0 사용)
#define MCPWM_UNIT MCPWM_UNIT_0
#define LEFT_TRACK_TIMER MCPWM_TIMER_0
#define RIGHT_TRACK_TIMER MCPWM_TIMER_1
#define TURRET_TIMER MCPWM_TIMER_2

// 스틱 데드존(-511~512)
#define STICK_INPUT_MIN (-511)
#define STICK_INPUT_MAX 512
#define STICK_DEAD_ZONE 80

// 모터 듀티(0~100) 데드존
#define MOTOR_DUTY_MIN 0
#define MOTOR_DUTY_MAX 100
#ifndef MOTOR_DEAD_ZONE
#define MOTOR_DEAD_ZONE 65
#endif
constexpr int MOTOR_ACTIVE_DUTY = 100 - MOTOR_DEAD_ZONE;

// 헤드라이트 PWM 설정
#define HEADLIGHT_PWM_CHANNEL 5
#define HEADLIGHT_PWM_FREQ 5000
#define HEADLIGHT_PWM_RESOLUTION 8
#define HEADLIGHT_DUTY_ON 128  // 50% 밝기 (255의 절반)
#define HEADLIGHT_DUTY_OFF 0
#define HEADLIGHT_DUTY_MIN 8
#define HEADLIGHT_DUTY_MAX 255
#define HEADLIGHT_DUTY_STEP 16

// 모터 설정 구조체
typedef struct {
  int in1Pin;
  int in2Pin;
  mcpwm_unit_t unit;
  mcpwm_timer_t timer;
  int& prevSpeed;
  bool& startBoostActive;
  unsigned long& startBoostStartTime;
} MotorConfig;

// EEPROM 주소

#define EEPROM_VOLUME_ADDR 4

// EEPROM 초기화 플래그
#define EEPROM_INIT_FLAG_ADDR 5

// 게임패드 관련 변수
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool gamepadConnected = false;

// DFPlayer 관련 변수
DFPlayerMini_Fast myDFPlayer;
HardwareSerial DFPlayerSerial(2); // UART2 사용
unsigned long lastIdleSoundTime = 0;
constexpr unsigned long idleSoundInterval = 13000; // 13초마다 효과음 1 재생

// 서보 모터 객체
Servo cannonMountServo; // 포 마운트 서보 모터
Servo cannonServo; // 포신 서보 모터

// 모터 제어 변수

int leftTrackPWM = 0;
int rightTrackPWM = 0;
int turretPWM = 0;
int cannonMountAngle = 90; // 포 마운트 기본 각도 (중앙)
constexpr int cannonAngle = 90; // 포신 기본 각도 (중앙)

// 볼륨 제어 변수
int currentVolume = 20; // 현재 볼륨 (1-30)
int tempVolume = 20; // 임시 볼륨 (버튼을 누르고 있는 동안 사용)
bool volumeChanged = false; // 볼륨이 변경되었는지 확인

// DC 모터 이전 속도 값 저장 변수
int prevLeftTrackSpeed = 0;
int prevRightTrackSpeed = 0;
int prevTurretSpeed = 0;

// 트랙 시동 부스트: 정지 후 500ms 동안 80% 듀티
constexpr int trackStartBoostDuty = 80;
constexpr unsigned long trackStartBoostDuration = 500;
bool leftTrackStartBoostActive = false;
bool rightTrackStartBoostActive = false;
unsigned long leftTrackStartBoostStartTime = 0;
unsigned long rightTrackStartBoostStartTime = 0;

// 모터 설정 구조체 인스턴스
MotorConfig leftTrackMotor = {.in1Pin = LEFT_TRACK_IN1,
                              .in2Pin = LEFT_TRACK_IN2,
                              .unit = MCPWM_UNIT,
                              .timer = LEFT_TRACK_TIMER,
                              .prevSpeed = prevLeftTrackSpeed,
                              .startBoostActive = leftTrackStartBoostActive,
                              .startBoostStartTime = leftTrackStartBoostStartTime};

MotorConfig rightTrackMotor = {.in1Pin = RIGHT_TRACK_IN1,
                               .in2Pin = RIGHT_TRACK_IN2,
                               .unit = MCPWM_UNIT,
                               .timer = RIGHT_TRACK_TIMER,
                               .prevSpeed = prevRightTrackSpeed,
                               .startBoostActive = rightTrackStartBoostActive,
                               .startBoostStartTime = rightTrackStartBoostStartTime};

boolean turretBoostActive = false;
unsigned long turretBoostStartTime = 0;
MotorConfig turretMotor = {.in1Pin = TURRET_IN1,
                           .in2Pin = TURRET_IN2,
                           .unit = MCPWM_UNIT,
                           .timer = TURRET_TIMER,
                           .prevSpeed = prevTurretSpeed,
                           .startBoostActive = turretBoostActive,
                           .startBoostStartTime = turretBoostStartTime};

// LED 상태
bool headlightOn = false;
int headlightDuty = 0; // 현재 헤드라이트 밝기 (0~255)
unsigned long lastBlinkTime = 0;
constexpr unsigned long blinkInterval = 100; // 100ms 간격으로 깜빡임

// 포신 발사 관련 변수
bool cannonFiring = false;
unsigned long cannonStartTime = 0;
constexpr unsigned long cannonDuration = 1000; // 1초 동안 포신 당김 (재발사 방지 쿨타임 역할)
constexpr unsigned long cannonLedDuration = 200; // LED는 200ms만 켜짐

// 기관총 발사 관련 변수
bool machineGunFiring = false;
unsigned long machineGunStartTime = 0;
constexpr unsigned long machineGunDuration = 600; // 1초간 기관총 발사

// 효과음 파일 번호
#define SOUND_IDLE 1
#define SOUND_CANNON 2
#define SOUND_MACHINEGUN 3
#define SOUND_CONNECTED 4

// 게임패드 연결 콜백
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      ESP_LOGI(MAIN_TAG, "Gamepad connected, index=%d", i);
      const ControllerProperties properties = ctl->getProperties();
      ESP_LOGI(MAIN_TAG, "Controller model: %s, VID=0x%04x, PID=0x%04x",
               ctl->getModelName().c_str(),
               properties.vendor_id,
               properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      gamepadConnected = true;

      // 게임 패드 진동
      ctl->playDualRumble(0, 300, 0xFF, 0x0);

      // 게임패드 연결 시 효과음 4 재생
      myDFPlayer.play(SOUND_CONNECTED);
      break;
    }
  }
  if (!foundEmptySlot) {
    ESP_LOGW(MAIN_TAG, "Gamepad connected, but no empty slot found");
  }
}

// 게임패드 연결 해제 콜백
void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      ESP_LOGI(MAIN_TAG, "Gamepad disconnected, index=%d", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  // 모든 게임패드가 연결 해제되었는지 확인
  gamepadConnected = false;
  for (const auto& myController : myControllers) {
    if (myController != nullptr) {
      gamepadConnected = true;
      break;
    }
  }

  if (!gamepadConnected) {
    // 모든 게임패드가 연결 해제되면 효과음 1 재생 시작
    myDFPlayer.play(SOUND_IDLE);
    lastIdleSoundTime = millis();
  }

  if (!foundController) {
    ESP_LOGW(MAIN_TAG, "Gamepad disconnected, but not found in myControllers");
  }
}

// DC 모터 제어 함수 (MCPWM 사용, 속도 변화가 없으면 호출 무시)
// stick : -511~512
// motor duty : 0~100
void setMotorSpeed(const MotorConfig& motor, const int stick, boolean force = false) {
  int duty = 0;
  const int absStick = abs(stick);
  if (absStick < STICK_DEAD_ZONE) {
    duty = MOTOR_DUTY_MIN;
  } else if (absStick > 500) {
    duty = stick > 0 ? MOTOR_DUTY_MAX : -MOTOR_DUTY_MAX;
  } else {
    const auto sanitizedStick = stick - STICK_DEAD_ZONE;
    duty = map(sanitizedStick,
               STICK_INPUT_MIN + STICK_DEAD_ZONE,
               STICK_INPUT_MAX - STICK_DEAD_ZONE,
               MOTOR_ACTIVE_DUTY * -1,
               MOTOR_ACTIVE_DUTY);
    if (duty > 0)
      duty += MOTOR_DEAD_ZONE;
    else
      duty -= MOTOR_DEAD_ZONE;
  }

  if (!force) {
    // 트랙 시동 부스트: 정지 후 움직이기 시작할 때 500ms 동안 80% 듀티로 가동
    if (motor.startBoostActive && motor.startBoostStartTime) {
      if (duty != 0 && motor.prevSpeed == 0) {
        motor.startBoostActive = true;
        motor.startBoostStartTime = millis();
      }
      const unsigned long now = millis();
      if (now - motor.startBoostStartTime < trackStartBoostDuration) {
        duty = (duty > 0) ? trackStartBoostDuty : -trackStartBoostDuty;
      } else {
        motor.startBoostActive = false;
      }
      if (duty == 0) {
        motor.startBoostActive = false;
      }
    }
  } else {
    motor.startBoostActive = false;
  }

  // 속도 변화가 없으면 호출 무시
  if (duty == motor.prevSpeed) {
    return;
  }

  ESP_LOGD(MAIN_TAG,
           "setMotorSpeed IN1:%d IN2:%d Unit:%d Timer:%d Stick:%d Duty:%3d (prev:%d)",
           motor.in1Pin, motor.in2Pin, motor.unit, motor.timer, stick, duty, motor.prevSpeed);

  if (duty > 0) {
    // 정방향 회전
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, static_cast<float>(duty));
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, 0);
  } else if (duty < 0) {
    // 역방향 회전
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, static_cast<float>(-duty));
  } else {
    // 정지
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_A, 0);
    mcpwm_set_duty(motor.unit, motor.timer, MCPWM_OPR_B, 0);
  }

  // 현재 속도를 이전 속도로 저장
  motor.prevSpeed = duty;
}

// 포 마운트 서보 모터 제어 함수
void setCannonMountAngle(const int angle) {
  ESP_LOGD(MAIN_TAG, "Cannon-Mount Angle %3d degree", angle);
  cannonMountServo.write(angle);
}

// 포신 서보 모터 제어 함수
void setCannonAngle(const int angle) {
  ESP_LOGD(MAIN_TAG, "Cannon Angle %3d degree", angle);
  cannonServo.write(angle);
}

// 볼륨 설정 저장
void saveVolumeSettings() {
  EEPROM.write(EEPROM_VOLUME_ADDR, currentVolume);
  EEPROM.commit();
  ESP_LOGI(MAIN_TAG, "Volume setting saved: %d", currentVolume);
}

// 볼륨 설정 로드
void loadVolumeSettings() {
  int volume = EEPROM.read(EEPROM_VOLUME_ADDR);

  // 기본값 설정 (EEPROM이 초기화되지 않은 경우)
  if (volume < 10 || volume > 30) {
    volume = 20; // 기본 볼륨 20
    EEPROM.write(EEPROM_VOLUME_ADDR, volume);
    EEPROM.commit();
  }

  currentVolume = volume;
  tempVolume = volume;
  myDFPlayer.volume(currentVolume);
  ESP_LOGI(MAIN_TAG, "Volume setting loaded: %d", currentVolume);
}

// EEPROM 초기화 및 ESP32 재시작
void resetEEPROMAndRestart() {
  ESP_LOGI(MAIN_TAG, "EEPROM 초기화 및 재시작 시작...");

  // 모든 EEPROM 데이터 초기화
  for (int i = 0; i < 512; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();

  ESP_LOGI(MAIN_TAG, "EEPROM 초기화 완료. 3초 후 재시작합니다.");

  // 3초 대기 후 재시작
  delay(3000);
  esp_restart();
}

void dumpGamepad(ControllerPtr ctl) {
  // clang-format off
  ESP_LOGV(MAIN_TAG,
           "0x%02x %s %s %s %s %s %s %s %s %s %s %s %s %s %s misc: 0x%02x LY:%3d RY:%3d",
           ctl->dpad(),
           ctl->a() ? "A" : "-",
           ctl->b() ? "B" : "-",
           ctl->x() ? "X" : "-",
           ctl->y() ? "Y" : "-",
           ctl->l1() ? "L1" : "--",
           ctl->r1() ? "R1" : "--",
           ctl->l2() ? "L2" : "--",
           ctl->r2() ? "R2" : "--",
           ctl->thumbL() ? "ThumbL" : "------",
           ctl->thumbR() ? "ThumbR" : "------",
           ctl->miscStart() ? "Start" : "------",
           ctl->miscSelect() ? "Select" : "------",
           ctl->miscSystem() ? "System" : "------",
           ctl->miscCapture() ? "Capture" : "------",
           ctl->miscButtons(),
           ctl->axisY(),
           ctl->axisRY());
  // clang-format on
}

// 게임패드 처리 함수
void processGamepad(ControllerPtr ctl) {
  dumpGamepad(ctl);

  // 좌측 스틱 Y축으로 좌측 트랙 제어, 우측 스틱 Y축으로 우측 트랙 제어

  // 모터 제어
  setMotorSpeed(leftTrackMotor, ctl->axisY());
  setMotorSpeed(rightTrackMotor, ctl->axisRY());

  // D-PAD로 터렛과 포 마운트 제어
  // D-PAD 좌우로 터렛 제어
  int turretSpeed = 0;
  if (ctl->dpad() == DPAD_LEFT) {
    turretSpeed = STICK_INPUT_MIN; // 좌측 회전
  } else if (ctl->dpad() == DPAD_RIGHT) {
    turretSpeed = STICK_INPUT_MAX; // 우측 회전
  }
  setMotorSpeed(turretMotor, turretSpeed);

  // D-PAD 상하로 포 마운트 각도 제어
  if (ctl->dpad() == DPAD_UP) {
    cannonMountAngle = constrain(cannonMountAngle + 2, 75, 160); // 위로 올림
    setCannonMountAngle(cannonMountAngle);
  } else if (ctl->dpad() == DPAD_DOWN) {
    cannonMountAngle = constrain(cannonMountAngle - 2, 75, 160); // 아래로 내림
    setCannonMountAngle(cannonMountAngle);
  }

  // B 버튼으로 포신 발사
  if (ctl->b() && !cannonFiring && !machineGunFiring) {
    cannonFiring = true;
    cannonStartTime = millis();
    // 포신 발사 중에는 LED를 지속 점등
    digitalWrite(CANNON_LED_PIN, HIGH);

    // 효과음 2 재생
    myDFPlayer.play(SOUND_CANNON);

    // 게임 패드 진동
    ctl->playDualRumble(0, 500, 0xFF, 0xFF);

    cannonServo.attach(CANNON_SERVO_PIN); // 포신 서보 모터
    delay(100);
    // 포신 당기기
    setCannonAngle(cannonAngle - 90); // 포신을 아래로 당김

    digitalWrite(CANNON_LED_PIN, LOW);

    delay(100);

    setMotorSpeed(leftTrackMotor, 256 + 64, true);
    setMotorSpeed(rightTrackMotor, 256 + 64, true);

    delay(100);

    setMotorSpeed(leftTrackMotor, 0);
    setMotorSpeed(rightTrackMotor, 0);

    delay(200);

    setCannonAngle(cannonAngle); // 포신을 원래 위치로 복원

    delay(300);
    cannonServo.detach(); // 포신 서보 모터
  }

  // A 버튼으로 기관총 발사
  if (ctl->a() && !machineGunFiring && !cannonFiring) {
    machineGunFiring = true;
    machineGunStartTime = millis();

    // 게임 패드 진동
    ctl->playDualRumble(0, 300, 0xFF, 0x0);

    // 효과음 3 재생
    myDFPlayer.play(SOUND_MACHINEGUN);
  }

  // L1 버튼으로 볼륨 조절 (둔감하게 처리)
  static bool l1ButtonPressed = false;
  static bool r1ButtonPressed = false;
  static unsigned long l1LastChangeTime = 0;
  static unsigned long r1LastChangeTime = 0;
  constexpr unsigned long volumeChangeInterval = 100; // 100ms 간격으로 볼륨 변경

  // L1 버튼으로 볼륨 감소 (Y 버튼이 눌리지 않았을 때만)
  if (ctl->l1() && !ctl->y()) {
    if (!l1ButtonPressed) {
      l1ButtonPressed = true;
      tempVolume = currentVolume; // 현재 볼륨을 임시 볼륨으로 복사
      l1LastChangeTime = millis();
    }

    // 볼륨 감소 (1-30 범위)
    if (tempVolume > 1 &&
        (millis() - l1LastChangeTime >= volumeChangeInterval)) {
      tempVolume--;
      l1LastChangeTime = millis();
      ESP_LOGI(MAIN_TAG, "Volume decreased to: %d", tempVolume);
    }
  } else if (!ctl->l1()) {
    // L1 버튼 해제 시에만 처리
    if (l1ButtonPressed) {
      l1ButtonPressed = false;
      // L1 버튼을 뗐을 때 볼륨 변경사항 저장
      if (tempVolume != currentVolume) {
        currentVolume = constrain(tempVolume, 10, 30);
        myDFPlayer.volume(currentVolume); // DFPlayer 볼륨 적용
        volumeChanged = true;
        ESP_LOGI(MAIN_TAG, "Volume change confirmed: %d", currentVolume);
      }
    }
  }

  // R1 버튼으로 볼륨 증가 (Y 버튼이 눌리지 않았을 때만)
  if (ctl->r1() && !ctl->y()) {
    if (!r1ButtonPressed) {
      r1ButtonPressed = true;
      tempVolume = currentVolume; // 현재 볼륨을 임시 볼륨으로 복사
      r1LastChangeTime = millis();
    }

    // 볼륨 증가 (1-30 범위)
    if (tempVolume < 30 &&
        (millis() - r1LastChangeTime >= volumeChangeInterval)) {
      tempVolume++;
      r1LastChangeTime = millis();
      ESP_LOGI(MAIN_TAG, "Volume increased to: %d", tempVolume);
    }
  } else if (!ctl->r1()) {
    // R1 버튼 해제 시에만 처리
    if (r1ButtonPressed) {
      r1ButtonPressed = false;
      // R1 버튼을 뗐을 때 볼륨 변경사항 저장
      if (tempVolume != currentVolume) {
        currentVolume = constrain(tempVolume, 10, 30);
        myDFPlayer.volume(currentVolume); // DFPlayer 볼륨 적용
        volumeChanged = true;
        ESP_LOGI(MAIN_TAG, "Volume change confirmed: %d", currentVolume);
      }
    }
  }

  // 볼륨이 변경되었으면 EEPROM에 저장
  if (volumeChanged) {
    saveVolumeSettings();
    volumeChanged = false;
  }

  // 헤드라이트 제어 관련 변수
  static bool yButtonPressed = false;
  static unsigned long lastBrightnessChangeTime = 0;

  // Y 버튼 처리 (밝기 조절 및 토글)
  if (ctl->y()) {
    // 1. 밝기 조절 (L1/R1과 조합)
    if (ctl->l1() || ctl->r1()) {
      constexpr unsigned long brightnessChangeInterval = 200;
      if (millis() - lastBrightnessChangeTime >= brightnessChangeInterval) {
        if (ctl->l1()) {
          headlightDuty = constrain(headlightDuty - HEADLIGHT_DUTY_STEP, HEADLIGHT_DUTY_MIN, HEADLIGHT_DUTY_MAX);
          ESP_LOGI(MAIN_TAG, "Headlight Brightness Decreased: %d", headlightDuty);
        }
        if (ctl->r1()) {
          headlightDuty = constrain(headlightDuty + HEADLIGHT_DUTY_STEP, HEADLIGHT_DUTY_MIN, HEADLIGHT_DUTY_MAX);
          ESP_LOGI(MAIN_TAG, "Headlight Brightness Increased: %d", headlightDuty);
        }
        // 현재 켜져있다면 즉시 적용
        if (headlightOn) {
          ledcWrite(HEADLIGHT_PWM_CHANNEL, headlightDuty);
        }
        lastBrightnessChangeTime = millis();
      }
      // 밝기 조절 중에는 토글 방지를 위해 pressed 상태 유지
      yButtonPressed = true;
    }
    // 2. 헤드라이트 토글 (단독 누름, 최초 눌림 시점)
    else if (!yButtonPressed) {
      yButtonPressed = true;
      headlightOn = !headlightOn;
      if (headlightOn && headlightDuty < HEADLIGHT_DUTY_MIN)
        headlightDuty = HEADLIGHT_DUTY_MIN;
      ledcWrite(HEADLIGHT_PWM_CHANNEL, headlightOn ? headlightDuty : HEADLIGHT_DUTY_OFF);
      ESP_LOGI(MAIN_TAG, "Headlight is %s (Duty: %d)", headlightOn ? "On" : "Off", headlightDuty);
    }
  } else {
    yButtonPressed = false;
  }

  // Select + Start 버튼 3초 이상 동시 누름으로 EEPROM 초기화 및 재시작
  static bool selectStartPressed = false;
  static unsigned long selectStartStartTime = 0;

  if (ctl->miscSelect() && ctl->miscStart()) {
    if (!selectStartPressed) {
      selectStartPressed = true;
      selectStartStartTime = millis();
      ESP_LOGI(MAIN_TAG, "Select + Start 버튼이 눌렸습니다. 3초간 유지하면 EEPROM 초기화가 시작됩니다.");
    } else {
      constexpr unsigned long selectStartHoldDuration = 3000;
      // 버튼이 계속 눌려있는 상태에서 3초 경과 확인
      if (millis() - selectStartStartTime >= selectStartHoldDuration) {
        ESP_LOGI(MAIN_TAG, "Select + Start 버튼을 3초간 누르셨습니다. EEPROM 초기화를 시작합니다.");

        // 게임패드 진동으로 확인 신호
        ctl->playDualRumble(0, 800, 0xFF, 0x0);

        // EEPROM 초기화 및 재시작
        resetEEPROMAndRestart();
      }
    }
  } else {
    selectStartPressed = false;
  }
}

// 포신 발사 처리
void processCannonFiring() {
  if (cannonFiring) {
    const unsigned long currentTime = millis();

    if (currentTime - cannonStartTime >= cannonDuration) {
      // 포신 발사 완료 (쿨타임 종료)
      cannonFiring = false;

      // 효과음 1 재생 재개 (게임패드가 연결되어 있지 않은 경우)
      if (!gamepadConnected && !machineGunFiring) {
        myDFPlayer.play(SOUND_IDLE);
        lastIdleSoundTime = millis();
      }
    }
  }
}

// 기관총 발사 처리
void processMachineGunFiring() {
  if (machineGunFiring) {
    const unsigned long currentTime = millis();
    if (currentTime - machineGunStartTime >= machineGunDuration) {
      // 기관총 발사 완료
      machineGunFiring = false;

      // 효과음 1 재생 재개 (게임패드가 연결되어 있지 않은 경우)
      if (!gamepadConnected && !cannonFiring) {
        myDFPlayer.play(SOUND_IDLE);
        lastIdleSoundTime = millis();
      }
    }
  }
}

// 효과음 반복 재생 처리
void processIdleSound() {
  if (!gamepadConnected && !cannonFiring && !machineGunFiring) {
    const unsigned long currentTime = millis();
    if (currentTime - lastIdleSoundTime >= idleSoundInterval) {
      myDFPlayer.play(SOUND_IDLE);
      lastIdleSoundTime = currentTime;
    }
  }
}

// 모든 컨트롤러 처리
void processControllers() {
  for (const auto myController : myControllers) {
    if (myController && myController->isConnected() &&
        myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      }
    }
  }
}

// 설정 함수
void setup() {
  ESP_LOGI(MAIN_TAG, "RC Tank Initialization...");

#ifdef DISABLE_BROWNOUT_DETECTOR
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable Brownout Detector
#endif

  // Brownout을 피하기 위해 CPU 클록을 160 MHz로 낮춤
  setCpuFrequencyMhz(160);

  // EEPROM 초기화
  EEPROM.begin(512);

  // 핀 모드 설정
  pinMode(LEFT_TRACK_IN1, OUTPUT);
  pinMode(LEFT_TRACK_IN2, OUTPUT);
  pinMode(RIGHT_TRACK_IN1, OUTPUT);
  pinMode(RIGHT_TRACK_IN2, OUTPUT);
  pinMode(TURRET_IN1, OUTPUT);
  pinMode(TURRET_IN2, OUTPUT);
  pinMode(CANNON_LED_PIN, OUTPUT);
  // pinMode(HEADLIGHT_PIN, OUTPUT); // LEDC로 제어하므로 주석 처리 또는 삭제

  // 헤드라이트 PWM 초기화
  ledcSetup(HEADLIGHT_PWM_CHANNEL, HEADLIGHT_PWM_FREQ, HEADLIGHT_PWM_RESOLUTION);
  ledcAttachPin(HEADLIGHT_PIN, HEADLIGHT_PWM_CHANNEL);
  ledcWrite(HEADLIGHT_PWM_CHANNEL, HEADLIGHT_DUTY_OFF);

  // MCPWM 설정
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 5000; // 5kHz
  pwm_config.cmpr_a = 0; // 초기 듀티 사이클 0%
  pwm_config.cmpr_b = 0; // 초기 듀티 사이클 0%
  pwm_config.counter_mode = MCPWM_UP_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

  // MCPWM 초기화 (모든 모터는 MCPWM_UNIT_0 사용)
  // 좌측 트랙 (TIMER_0)
  mcpwm_init(MCPWM_UNIT, LEFT_TRACK_TIMER, &pwm_config);
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM0A, LEFT_TRACK_IN1);
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM0B, LEFT_TRACK_IN2);

  // 우측 트랙 (TIMER_1)
  mcpwm_init(MCPWM_UNIT, RIGHT_TRACK_TIMER, &pwm_config);
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM1A, RIGHT_TRACK_IN1);
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM1B, RIGHT_TRACK_IN2);

  // 터렛 (TIMER_2)
  mcpwm_init(MCPWM_UNIT, TURRET_TIMER, &pwm_config);
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM2A, TURRET_IN1);
  mcpwm_gpio_init(MCPWM_UNIT, MCPWM2B, TURRET_IN2);

  // 모터 정지
  setMotorSpeed(leftTrackMotor, 0);
  setMotorSpeed(rightTrackMotor, 0);
  setMotorSpeed(turretMotor, 0);

  // 서보 모터 초기화
  cannonMountServo.attach(CANNON_MOUNT_SERVO_PIN); // 포 마운트 서보 모터
  // cannonServo.attach(CANNON_SERVO_PIN); // 포신 서보 모터

  // 서보 모터 초기 위치 설정
  setCannonMountAngle(cannonMountAngle);
  setCannonAngle(cannonAngle);

  // DFPlayer 초기화
  DFPlayerSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
  myDFPlayer.begin(DFPlayerSerial);
  // 볼륨은 loadVolumeSettings()에서 설정됨

  // EEPROM에서 설정 로드

  loadVolumeSettings();

  // 효과음 1 재생 시작
  myDFPlayer.play(SOUND_IDLE);
  lastIdleSoundTime = millis();

  // EEPROM 초기화 플래그 확인 (첫 실행 시)
  const int initFlag = EEPROM.read(EEPROM_INIT_FLAG_ADDR);
  if (initFlag != 0xAA) {
    ESP_LOGI(MAIN_TAG, "EEPROM이 초기화되지 않았습니다. 초기화 플래그를 설정합니다.");
    EEPROM.write(EEPROM_INIT_FLAG_ADDR, 0xAA);
    EEPROM.commit();
  }

  // Bluepad32 설정
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);

  ESP_LOGI(MAIN_TAG, "Firmware version: %s", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  ESP_LOGI(MAIN_TAG, "BD address: %2X:%2X:%2X:%2X:%2X:%2X",
           addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  ESP_LOGI(MAIN_TAG, "RC Tank Initialization Complete!");
}

// 메인 루프
void loop() {
  // Bluepad32 업데이트
  const bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }

  // 포신 발사 처리
  processCannonFiring();

  // 기관총 발사 처리
  processMachineGunFiring();

  // 효과음 반복 재생 처리
  processIdleSound();

  delay(10); // 10ms 딜레이
}
