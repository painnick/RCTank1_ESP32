#include <Arduino.h>
#include <Bluepad32.h>
#include <DFPlayerMini_Fast.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <esp_log.h>
#include <driver/mcpwm.h>

static const char* MAIN_TAG = "RC_TANK";

// 핀 정의
#define DFPLAYER_RX 32
#define DFPLAYER_TX 33
#define LEFT_TRACK_IN1 25
#define LEFT_TRACK_IN2 26
#define RIGHT_TRACK_IN1 27
#define RIGHT_TRACK_IN2 13
#define TURRET_IN1 12
#define TURRET_IN2 14
#define LED_PIN 2
#define HEADLIGHT_PIN 4
#define CANNON_MOUNT_SERVO_PIN 5   // 포 마운트 서보 모터 핀 (우측 Y축으로 각도 조절)
#define CANNON_SERVO_PIN 18     // 포신 서보 모터 핀 (A 버튼으로 당기기)

// MCPWM 유닛 및 타이머 정의
#define LEFT_TRACK_MCPWM_UNIT MCPWM_UNIT_0
#define RIGHT_TRACK_MCPWM_UNIT MCPWM_UNIT_0
#define TURRET_MCPWM_UNIT MCPWM_UNIT_1

#define LEFT_TRACK_MCPWM_TIMER MCPWM_TIMER_0
#define RIGHT_TRACK_MCPWM_TIMER MCPWM_TIMER_1
#define TURRET_MCPWM_TIMER MCPWM_TIMER_2

// DC 모터 최소 속도 임계값 (50 미만은 처리하지 않음)
#define MOTOR_MIN_SPEED_THRESHOLD 50

// EEPROM 주소
#define EEPROM_LEFT_SPEED_ADDR 0
#define EEPROM_RIGHT_SPEED_ADDR 1

// 게임패드 관련 변수
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
bool gamepadConnected = false;

// DFPlayer 관련 변수
DFPlayerMini_Fast myDFPlayer;
HardwareSerial DFPlayerSerial(2); // UART2 사용
unsigned long lastIdleSoundTime = 0;
const unsigned long idleSoundInterval = 13000; // 13초마다 효과음 1 재생

// 서보 모터 객체
Servo cannonMountServo;  // 포 마운트 서보 모터
Servo cannonServo;     // 포신 서보 모터

// 모터 제어 변수
int leftTrackSpeed = 255;  // 기본값
int rightTrackSpeed = 255; // 기본값
int leftTrackPWM = 0;
int rightTrackPWM = 0;
int turretPWM = 0;
int cannonMountAngle = 90; // 포 마운트 기본 각도 (중앙)
int cannonAngle = 90;    // 포신 기본 각도 (중앙)

// DC 모터 이전 속도 값 저장 변수
int prevLeftTrackSpeed = 0;
int prevRightTrackSpeed = 0;
int prevTurretSpeed = 0;

// LED 상태
bool headlightOn = false;
bool ledBlinking = false;
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 100; // 100ms 간격으로 깜빡임

// 포신 발사 관련 변수
bool cannonFiring = false;
unsigned long cannonStartTime = 0;
const unsigned long cannonDuration = 500; // 500ms 동안 포신 당김

// 기관총 발사 관련 변수
bool machinegunFiring = false;
unsigned long machinegunStartTime = 0;
const unsigned long machinegunDuration = 3000; // 3초간 기관총 발사

// 효과음 파일 번호
#define SOUND_IDLE 1
#define SOUND_CANNON 2
#define SOUND_MACHINEGUN 3

// 게임패드 연결 콜백
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            ESP_LOGI(MAIN_TAG, "Gamepad connected, index=%d", i);
            ControllerProperties properties = ctl->getProperties();
            ESP_LOGI(MAIN_TAG, "Controller model: %s, VID=0x%04x, PID=0x%04x",
                     ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            gamepadConnected = true;

            // 게임패드 연결 시 효과음 1 중단
            myDFPlayer.stop();
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
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr) {
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
void setMotorSpeed(int in1Pin, int in2Pin, mcpwm_unit_t unit, mcpwm_timer_t timer, int speed, int* prevSpeed) {
    // 최소 속도 임계값 적용
    if (abs(speed) < MOTOR_MIN_SPEED_THRESHOLD) {
        speed = 0;
    }

    // 속도 변화가 없으면 호출 무시
    if (speed == *prevSpeed) {
        return;
    }

    ESP_LOGD(MAIN_TAG, "setMotorSpeed IN1:%d IN2:%d Unit:%d Timer:%d Speed:%d (prev:%d)",
             in1Pin, in2Pin, unit, timer, speed, *prevSpeed);

    if (speed > 0) {
        // 정방향 회전
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, speed);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);
    } else if (speed < 0) {
        // 역방향 회전
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, -speed);
    } else {
        // 정지
        mcpwm_set_duty(unit, timer, MCPWM_OPR_A, 0);
        mcpwm_set_duty(unit, timer, MCPWM_OPR_B, 0);
    }

    // 현재 속도를 이전 속도로 저장
    *prevSpeed = speed;
}

// 포 마운트 서보 모터 제어 함수
void setCannonMountAngle(int angle) {
    cannonMountServo.write(angle);
}

// 포신 서보 모터 제어 함수
void setCannonAngle(int angle) {
    cannonServo.write(angle);
}

// EEPROM에서 속도 값 읽기
void loadSpeedSettings() {
    leftTrackSpeed = EEPROM.read(EEPROM_LEFT_SPEED_ADDR);
    rightTrackSpeed = EEPROM.read(EEPROM_RIGHT_SPEED_ADDR);

    // 기본값 설정 (EEPROM이 초기화되지 않은 경우)
    if (leftTrackSpeed == 0 || leftTrackSpeed > 255) {
        leftTrackSpeed = 255;
        EEPROM.write(EEPROM_LEFT_SPEED_ADDR, leftTrackSpeed);
    }
    if (rightTrackSpeed == 0 || rightTrackSpeed > 255) {
        rightTrackSpeed = 255;
        EEPROM.write(EEPROM_RIGHT_SPEED_ADDR, rightTrackSpeed);
    }
    EEPROM.commit();

    ESP_LOGI(MAIN_TAG, "Loaded speed settings: left=%d, right=%d", leftTrackSpeed, rightTrackSpeed);
}

// EEPROM에 속도 값 저장
void saveSpeedSettings() {
    EEPROM.write(EEPROM_LEFT_SPEED_ADDR, leftTrackSpeed);
    EEPROM.write(EEPROM_RIGHT_SPEED_ADDR, rightTrackSpeed);
    EEPROM.commit();
    ESP_LOGI(MAIN_TAG, "Speed settings saved: left=%d, right=%d", leftTrackSpeed, rightTrackSpeed);
}

// 게임패드 처리 함수
void processGamepad(ControllerPtr ctl) {
    // 좌측 스틱 Y축으로 좌측 트랙 제어, 우측 스틱 Y축으로 우측 트랙 제어
    int leftStickY = ctl->axisY();
    int rightStickY = ctl->axisRY();

    // 데드존 설정
    if (abs(leftStickY) < 50) leftStickY = 0;
    if (abs(rightStickY) < 50) rightStickY = 0;

    // 좌측 스틱 Y축으로 좌측 트랙 전후진 제어
    int leftTrackSpeed = map(leftStickY, -512, 512, -255, 255);
    
    // 우측 스틱 Y축으로 우측 트랙 전후진 제어
    int rightTrackSpeed = map(rightStickY, -512, 512, -255, 255);

    // 속도 제한
    leftTrackSpeed = constrain(leftTrackSpeed, -255, 255);
    rightTrackSpeed = constrain(rightTrackSpeed, -255, 255);

    // L1, L2로 좌측 트랙 속도 조절
    if (ctl->l1()) {
        leftTrackSpeed = map(leftTrackSpeed, -255, 255, -leftTrackSpeed, leftTrackSpeed);
    }
    if (ctl->l2() > 100) {
        int l2Value = map(ctl->l2(), 0, 1023, 0, 255);
        leftTrackSpeed = map(leftTrackSpeed, -255, 255, -l2Value, l2Value);
    }

    // R1, R2로 우측 트랙 속도 조절
    if (ctl->r1()) {
        rightTrackSpeed = map(rightTrackSpeed, -255, 255, -rightTrackSpeed, rightTrackSpeed);
    }
    if (ctl->r2() > 100) {
        int r2Value = map(ctl->r2(), 0, 1023, 0, 255);
        rightTrackSpeed = map(rightTrackSpeed, -255, 255, -r2Value, r2Value);
    }

    // 모터 제어
    setMotorSpeed(LEFT_TRACK_IN1, LEFT_TRACK_IN2, LEFT_TRACK_MCPWM_UNIT, LEFT_TRACK_MCPWM_TIMER, leftTrackSpeed, &prevLeftTrackSpeed);
    setMotorSpeed(RIGHT_TRACK_IN1, RIGHT_TRACK_IN2, RIGHT_TRACK_MCPWM_UNIT, RIGHT_TRACK_MCPWM_TIMER, rightTrackSpeed, &prevRightTrackSpeed);

    // D-PAD로 터렛과 포 마운트 제어
    // D-PAD 좌우로 터렛 제어
    int turretSpeed = 0;
    if (ctl->dpadLeft()) {
        turretSpeed = -255; // 좌측 회전
    } else if (ctl->dpadRight()) {
        turretSpeed = 255;  // 우측 회전
    }
    setMotorSpeed(TURRET_IN1, TURRET_IN2, TURRET_MCPWM_UNIT, TURRET_MCPWM_TIMER, turretSpeed, &prevTurretSpeed);

    // D-PAD 상하로 포 마운트 각도 제어
    if (ctl->dpadUp()) {
        cannonMountAngle = constrain(cannonMountAngle + 2, 0, 180); // 위로 올림
        setCannonMountAngle(cannonMountAngle);
    } else if (ctl->dpadDown()) {
        cannonMountAngle = constrain(cannonMountAngle - 2, 0, 180); // 아래로 내림
        setCannonMountAngle(cannonMountAngle);
    }

    // A 버튼으로 포신 발사
    if (ctl->a() && !cannonFiring && !machinegunFiring) {
        cannonFiring = true;
        cannonStartTime = millis();
        ledBlinking = true;

        // 게임 패드 진동
        ctl->playDualRumble(0, 400, 0xFF , 0x0);

        // 포신 당기기
        setCannonAngle(45); // 포신을 아래로 당김

        // 효과음 2 재생
        myDFPlayer.play(SOUND_CANNON);
    }

    // B 버튼으로 기관총 발사
    if (ctl->b() && !machinegunFiring && !cannonFiring) {
        machinegunFiring = true;
        machinegunStartTime = millis();
        ledBlinking = true;

        // 게임 패드 진동
        ctl->playDualRumble(0, 200, 0xFF , 0x0);

        // 효과음 3 재생
        myDFPlayer.play(SOUND_MACHINEGUN);
    }

    // X 버튼으로 헤드라이트 토글
    static bool xButtonPressed = false;
    if (ctl->x() && !xButtonPressed) {
        headlightOn = !headlightOn;
        digitalWrite(HEADLIGHT_PIN, headlightOn ? HIGH : LOW);
        xButtonPressed = true;
    } else if (!ctl->x()) {
        xButtonPressed = false;
    }

    // L1, L2로 좌측 트랙 속도 설정 저장
    static bool l1Pressed = false, l2Pressed = false;
    if (ctl->l1() && !l1Pressed) {
        leftTrackSpeed = constrain(leftTrackSpeed + 10, 0, 255);
        saveSpeedSettings();
        l1Pressed = true;
    } else if (!ctl->l1()) {
        l1Pressed = false;
    }

    if (ctl->l2() > 100 && !l2Pressed) {
        leftTrackSpeed = constrain(leftTrackSpeed - 10, 0, 255);
        saveSpeedSettings();
        l2Pressed = true;
    } else if (ctl->l2() <= 100) {
        l2Pressed = false;
    }

    // R1, R2로 우측 트랙 속도 설정 저장
    static bool r1Pressed = false, r2Pressed = false;
    if (ctl->r1() && !r1Pressed) {
        rightTrackSpeed = constrain(rightTrackSpeed + 10, 0, 255);
        saveSpeedSettings();
        r1Pressed = true;
    } else if (!ctl->r1()) {
        r1Pressed = false;
    }

    if (ctl->r2() > 100 && !r2Pressed) {
        rightTrackSpeed = constrain(rightTrackSpeed - 10, 0, 255);
        saveSpeedSettings();
        r2Pressed = true;
    } else if (ctl->r2() <= 100) {
        r2Pressed = false;
    }
}

// 포신 발사 처리
void processCannonFiring() {
    if (cannonFiring) {
        unsigned long currentTime = millis();
        if (currentTime - cannonStartTime >= cannonDuration) {
            // 포신 발사 완료
            cannonFiring = false;
            
            // 기관총이 발사 중이 아닌 경우에만 LED 점멸 중단
            if (!machinegunFiring) {
                ledBlinking = false;
                digitalWrite(LED_PIN, LOW);
            }

            // 포신을 원래 각도로 복원
            setCannonAngle(cannonAngle);

            // 효과음 1 재생 재개 (게임패드가 연결되어 있지 않은 경우)
            if (!gamepadConnected && !machinegunFiring) {
                myDFPlayer.play(SOUND_IDLE);
                lastIdleSoundTime = millis();
            }
        }
    }
}

// 기관총 발사 처리
void processMachinegunFiring() {
    if (machinegunFiring) {
        unsigned long currentTime = millis();
        if (currentTime - machinegunStartTime >= machinegunDuration) {
            // 기관총 발사 완료
            machinegunFiring = false;
            
            // 포신이 발사 중이 아닌 경우에만 LED 점멸 중단
            if (!cannonFiring) {
                ledBlinking = false;
                digitalWrite(LED_PIN, LOW);
            }

            // 효과음 1 재생 재개 (게임패드가 연결되어 있지 않은 경우)
            if (!gamepadConnected && !cannonFiring) {
                myDFPlayer.play(SOUND_IDLE);
                lastIdleSoundTime = millis();
            }
        }
    }
}

// LED 깜빡임 처리
void processLEDBlinking() {
    if (ledBlinking) {
        unsigned long currentTime = millis();
        
        // 기관총 발사 중일 때는 500ms 간격으로 점멸
        if (machinegunFiring) {
            if (currentTime - lastBlinkTime >= 500) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                lastBlinkTime = currentTime;
            }
        } else {
            // 포신 발사 중일 때는 기존 100ms 간격으로 점멸
            if (currentTime - lastBlinkTime >= blinkInterval) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                lastBlinkTime = currentTime;
            }
        }
    }
}

// 효과음 반복 재생 처리
void processIdleSound() {
    if (!gamepadConnected && !cannonFiring && !machinegunFiring) {
        unsigned long currentTime = millis();
        if (currentTime - lastIdleSoundTime >= idleSoundInterval) {
            myDFPlayer.play(SOUND_IDLE);
            lastIdleSoundTime = currentTime;
        }
    }
}

// 모든 컨트롤러 처리
void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

// 설정 함수
void setup() {
    // Serial.begin(115200);
    ESP_LOGI(MAIN_TAG, "RC Tank Initialization...");

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
    pinMode(LED_PIN, OUTPUT);
    pinMode(HEADLIGHT_PIN, OUTPUT);

    // MCPWM 설정
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 5000;    // 5kHz
    pwm_config.cmpr_a = 0;          // 초기 듀티 사이클 0%
    pwm_config.cmpr_b = 0;          // 초기 듀티 사이클 0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // 좌측 트랙 MCPWM 설정
    mcpwm_init(LEFT_TRACK_MCPWM_UNIT, LEFT_TRACK_MCPWM_TIMER, &pwm_config);
    mcpwm_gpio_init(LEFT_TRACK_MCPWM_UNIT, MCPWM0A, LEFT_TRACK_IN1);
    mcpwm_gpio_init(LEFT_TRACK_MCPWM_UNIT, MCPWM0B, LEFT_TRACK_IN2);

    // 우측 트랙 MCPWM 설정 (다른 유닛 사용)
    mcpwm_init(RIGHT_TRACK_MCPWM_UNIT, RIGHT_TRACK_MCPWM_TIMER, &pwm_config);
    mcpwm_gpio_init(RIGHT_TRACK_MCPWM_UNIT, MCPWM0A, RIGHT_TRACK_IN1);
    mcpwm_gpio_init(RIGHT_TRACK_MCPWM_UNIT, MCPWM0B, RIGHT_TRACK_IN2);

    // 터렛 MCPWM 설정 (LEFT_TRACK와 같은 유닛, 다른 타이머 사용)
    mcpwm_init(TURRET_MCPWM_UNIT, TURRET_MCPWM_TIMER, &pwm_config);
    mcpwm_gpio_init(TURRET_MCPWM_UNIT, MCPWM2A, TURRET_IN1);
    mcpwm_gpio_init(TURRET_MCPWM_UNIT, MCPWM2B, TURRET_IN2);

    // 모터 정지
    setMotorSpeed(LEFT_TRACK_IN1, LEFT_TRACK_IN2, LEFT_TRACK_MCPWM_UNIT, LEFT_TRACK_MCPWM_TIMER, 0, &prevLeftTrackSpeed);
    setMotorSpeed(RIGHT_TRACK_IN1, RIGHT_TRACK_IN2, RIGHT_TRACK_MCPWM_UNIT, RIGHT_TRACK_MCPWM_TIMER, 0, &prevRightTrackSpeed);
    setMotorSpeed(TURRET_IN1, TURRET_IN2, TURRET_MCPWM_UNIT, TURRET_MCPWM_TIMER, 0, &prevTurretSpeed);

    // 서보 모터 초기화
    cannonMountServo.attach(CANNON_MOUNT_SERVO_PIN);  // 포 마운트 서보 모터
    cannonServo.attach(CANNON_SERVO_PIN);        // 포신 서보 모터

    // 서보 모터 초기 위치 설정
    setCannonMountAngle(cannonMountAngle);
    setCannonAngle(cannonAngle);

    // DFPlayer 초기화
    DFPlayerSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);
    myDFPlayer.begin(DFPlayerSerial);
    myDFPlayer.volume(20); // 볼륨 설정 (0-30)

    // 효과음 1 재생 시작
    myDFPlayer.play(SOUND_IDLE);
    lastIdleSoundTime = millis();

    // EEPROM에서 속도 설정 로드
    loadSpeedSettings();

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
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }

    // 포신 발사 처리
    processCannonFiring();

    // 기관총 발사 처리
    processMachinegunFiring();

    // LED 깜빡임 처리
    processLEDBlinking();

    // 효과음 반복 재생 처리
    processIdleSound();

    delay(10); // 10ms 딜레이
}
