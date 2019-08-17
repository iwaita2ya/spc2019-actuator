#define DEBUG

#include <mbed.h>
#include "SystemParameters.h"
#include "GsLSM9DS1.h"
#include "SerialSRAM.h"
#include "sineTable.h"
#include "SensorManager.h"
#include "MotorManager.h"


// sensor update frequency
#define UPDATE_SENSOR_FREQ 0.1f // センサ更新間隔(秒) //TODO: 本番運用時 <0.1
// SRAM
#define CONFIG_AREA_SIZE 0x20   // 0x00-0x20
#define CONFIG_LOG_SIZE  0x09   // Status(1)+gz_raw(2),aDutyIndex(1),bDutyIndex(1),aRPM(2),bRPM(2),
#define LOG_START_AT   0x0020   // Status + Altitude = 2byte
#define SRAM_MAX_SIZE  0x0800   // 0x0000-0x0800

using namespace greysound;

// 状態管理
enum DeviceState {
    IDLE     = 0x01,
    ACTIVE   = 0x02,
    ERROR    = 0x80
};

/**
 * params
 */
static uint8_t aDutyIndex = 0;
static uint8_t bDutyIndex = 0;

/**
 * Buttons
 */
InterruptIn *enableSensors;    // センサ開始トリガ

/**
 * flags
 */
uint8_t isActive;


/**
 * Serial Port
 */
RawSerial *serial; // tx, rx
// Circular buffers for serial TX and RX data - used by interrupt routines
const int serialBufferSize = 255;

// might need to increase buffer size for high baud rates
char rxBuffer[serialBufferSize+1];

// Circular buffer pointers
// volatile makes read-modify-write atomic
volatile int rxInPointer=0;
volatile int rxOutPointer=0;

// Line buffers for sprintf and sscanf
char rxLineBuffer[80];

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) serial->printf(fmt, __VA_ARGS__)
#define DEBUG_PUTC(x) serial->putc(x)
#else
#define DEBUG_PRINT(fmt, ...)
#define DEBUG_PUTC(x)
#endif


/**
 * LED
 */
DigitalOut *led; // LED

/**
 * SRAM
 */
SerialSRAM *sram;

/**
 * Config
 * 設定情報を格納する
 */
SystemParameters *config;

using namespace greysound;

/**
 * Sensor Manager
 */
Ticker *sensorTicker;   // センサ更新タイマ
SensorManager *sensorManager;

/**
 * Motor Manager
 */
MotorManager *motorManager;

// Function Prototypes --------------------------------------------------------
// Serial
void interruptRx();
void readLine();

// アクチュエータ状態遷移
static void startFlywheel();
static void stopFlywheel();

// センサ
void calibrateSensor();
static void updateSensor();

// エラー表示
static void indicateError();

// ----- SRAM -----
void printConfig();         // get config data from SRAM (hex)
void printConfigReadable(); // get config data from SRAM (ascii)
void loadConfig();          // load config data from SRAM to Vars
void saveConfig();          // Save config data onto SRAM
void resetConfig();         // Init config with default value (and save onto SRAM)
void dumpMemory();          // dump all data in SRAM (hex)
void dumpMemoryReadable();  // dump all data in SRAM (ascii)
void clearLog(uint16_t startAddress=CONFIG_AREA_SIZE, uint16_t endAddress=SRAM_MAX_SIZE); // clear logged data
void logData(uint8_t _currentStatus, int16_t gz_raw, uint8_t aDutyIndex, uint8_t bDutyIndex, uint16_t aRPM, uint16_t bRPM);

// TEST
void dutyTest();
void brakeTest();

// Main  ----------------------------------------------------------------------

/**
 * Main
 * @return
 */
int main() {

    //-------------------------------------
    // Init
    //-------------------------------------

    //TODO: get actual time from RTC
    set_time(1546268400);  // 2019-01-01 00:00:00

    // set active flag
    isActive = 1;

    /**
     * Init Serial
     */
    serial = new RawSerial(P0_19, P0_18); // tx, rx
    serial->baud(115200); // default:9600bps
    serial->attach(&interruptRx, Serial::RxIrq); // interrupts for Rx

    /**
     * init SensorManager (and Ticker)
     */
    DEBUG_PRINT("Init SensorManager\r\n", NULL);
    sensorTicker = new Ticker();
    sensorManager = new SensorManager(P0_5, P0_4, 0xD6, 0x3C); // sda, scl, agAddr, mAddr, LED1
    sensorManager->init();

    /**
     * Init MotorManager
     * forward, reverse, standBy, aIn1, aIn2, bIn1, bIn2, aCount, bCount
     */
    DEBUG_PRINT("Init MotorManager\r\n", NULL);
    motorManager = new MotorManager(P0_9, P0_10, P0_8, P0_11, P0_12, P0_13, P0_14, P0_15, P0_16);

    /**
     * init SRAM
     */
    DEBUG_PRINT("Init SRAM\r\n", NULL);
    sram = new SerialSRAM(P0_5, P0_4, P0_6); // sda, scl, hs, A2=0, A1=0
    sram->setAutoStore(1); // enable auto-store

    /**
     * init System Params
     */
    config = new SystemParameters();
//    resetConfig(); //MEMO: 設定初期化 (DEBUG ONLY)
    loadConfig(); // 設定をSRAMから変数に読込

    /**
     * Init Buttons
     */
    enableSensors = new InterruptIn(P0_17);     // センサ開始／停止信号
    enableSensors->mode(PullUp);
    enableSensors->fall(&startFlywheel);
    enableSensors->rise(&stopFlywheel);

    /**
     * Init LED
     */
    led = new DigitalOut(P0_7);
    led->write(0); // set led off

    //-------------------------------------
    // Main Procedure
    //-------------------------------------

    // start motor
    DEBUG_PRINT("Start Motor\r\n", NULL);
    motorManager->start();

    //MEMO: TEST TEST TEST !!!
//    DEBUG_PRINT("Start DUTY TEST\r\n");
//    dutyTest();
//    DEBUG_PRINT("Start Brake Test\r\n");
//    brakeTest();

    // キャリブレーションが終わっていない場合は実行する
    if(config->gyroBiasRawX == 0 && config->gyroBiasRawY == 0 && config->gyroBiasRawZ == 0) {
        calibrateSensor();
    }

    // デバイスの状態に応じでアクチュエータを起動・停止する
    if(config->statusFlags == IDLE) {
        DEBUG_PRINT("Stop Flywheel\r\n", NULL);
        stopFlywheel();
    } else if (config->statusFlags == ACTIVE) {
        DEBUG_PRINT("Start Flywheel\r\n", NULL);
        startFlywheel();
    }

    DEBUG_PRINT("Start Main Loop\r\n", NULL);
    while(isActive == 1) {

        /**
         * シリアルコマンド応答
         * (アイドル時のみ有効)
         */
        // data received and not read yet?
//        if (config->statusFlags == IDLE && rxInPointer != rxOutPointer) {
        if (rxInPointer != rxOutPointer) {

            readLine();
            char commandByte = rxLineBuffer[0];

            switch (commandByte) {
                case 0x00: // キャリブレーション
                    calibrateSensor();
                    break;
                case 0x10: // アクチュエータ停止
                    stopFlywheel();
                    break;
                case 0x11: // アクチュエータ開始
                    startFlywheel();
                    break;
                case 0x20: // Config 表示 (hex)
                    printConfig();
                    break;
                case 0x21: // Config 表示 (ascii)
                    printConfigReadable();
                    break;
                case 0x22: // Config 初期化
                    resetConfig();
                    break;
                case 0x23: // Load SRAM->Config
                    loadConfig();
                    break;
                case 0x24: // Save Config->SRAM
                    saveConfig();
                    break;
                case 0x30: // ログデータ消去
                    clearLog();
                    break;
                case 0x40: // メモリダンプ　(hex)
                    dumpMemory();
                    break;
                case 0x41: // メモリダンプ　(ascii)
                    dumpMemoryReadable();
                    break;
                case 0xF0: // 力率あたりの回転数を測定
                    dutyTest();
                    break;
                case 0xF1: // ブレーキテスト
                    brakeTest();
                    break;
                default:
                    break;
            }
        }


        // RPM を表示
//                motorManager->read();
//        serial->printf("duration: %lu aCounter:%ld bCounter:%ld aCount:%ld bCount:%ld aRPM: %d bRPM %d\r\n"
//                , motorManager->duration
//                , motorManager->aCounter, motorManager->bCounter
//                , motorManager->aCount, motorManager->bCount
//                , (int)motorManager->aRPM, (int)motorManager->bRPM);

        // 角度を表示
//        serial->printf("duration: %lu aCounter:%d aRPM: %d bRPM %d\r\n"
//                , motorManager->duration
//                , (int)sensorManager->lsm9dof->gz
//                , (int)motorManager->aRPM, (int)motorManager->bRPM);

        led->write(!(led->read()));
        wait(0.5);
    }

    // stop & terminate objects
    motorManager->stop();
    stopFlywheel();
    delete(sensorManager);
    delete(sensorTicker);
    delete(sram);
    delete(serial);

    return 0;
}

/**
 * Functions
 */

static void startFlywheel()
{
    // センサ開始
    if(sensorManager && sensorManager->getCurrentState() == SensorManager::STAND_BY)
    {
        // failed to start
        if (sensorManager->begin() == 0) {
            indicateError();
            return; //TODO: エラーからリカバリできるようにする
        }

        // センサ値更新処理開始
        if(sensorTicker) {
            sensorTicker->attach(&updateSensor, UPDATE_SENSOR_FREQ);
        }

        // フラグ更新&Save
        config->statusFlags = ACTIVE;
        sram->write(0x0000, config->statusFlags);
        config->enableLogging = 1;
        sram->write(0x0007, config->enableLogging);
    }
}


static void stopFlywheel()
{
    // センサ値更新処理停止
    if(sensorTicker) {
        sensorTicker->detach();
    }

    // センサ停止
    if(sensorManager && sensorManager->getCurrentState() != SensorManager::STAND_BY) {
        sensorManager->stop();
    }

    // モータ停止（BRAKE ではなく STOP で空転させる）
    motorManager->changeMotorState(MOTOR_A, STOP);
    motorManager->changeMotorState(MOTOR_A, STOP);

    // フラグ更新
    if(config->statusFlags == ACTIVE) {
        config->statusFlags = IDLE;
        sram->write(0x0000, config->statusFlags);
        config->enableLogging = 0;
        sram->write(0x0007, config->enableLogging);

        // save data onto EEPROM
        sram->callHardwareStore();
    }
}


void calibrateSensor() {
    DEBUG_PRINT("Perform Calibration\r\n", NULL);
    sensorManager->calibration(&config->gyroBiasRawX, &config->gyroBiasRawY, &config->gyroBiasRawZ);
    saveConfig();
}

/**
 * センサの値に応じてサーボの回転方向とDuty比を調整する
 */
static void updateSensor()
{
    static int16_t gz_raw = 0;
    static int8_t gz128=0, gz128Last=0;
    static uint8_t indexDegree=1;
    motorState aMotorDirection, bMotorDirection;

    if(sensorManager) {

        // read sensor values
        sensorManager->read();

        // get gyro z-axis rotation value
        gz_raw = sensorManager->lsm9dof->gz_raw; // -32768<->32767

        // MEMO: 256(-128<->127)段階に変換
//        gz128 = uint8_t ((gz_raw >> 8) * 0.8 + gz128Last * 0.3); // -128<->127
        gz128 = (gz_raw >> 8); // -128<->127

        // モータの回転状態を取得(FORWARD|REVERSE|STOP|BRAKE)
        aMotorDirection = motorManager->getMotorState(MOTOR_A);
        bMotorDirection = motorManager->getMotorState(MOTOR_B);

        /**
         * プローブは正方向に回転
         */
        if(gz128 > 0) {

            // モータの回転方向に応じて処理分岐(MotorA)
            switch (aMotorDirection) {
                case FORWARD: { // 正方向（＝プローブの回転方向と等しい）

                    // 差が10以上離れている場合はインデックスを一気に10増加させる
                    indexDegree = (gz128 - aDutyIndex >= 10) ? 10 : 1;

                    if(gz128 > aDutyIndex) {
                        // 加速
                        aDutyIndex += indexDegree;
                    } else if (gz128 < aDutyIndex){
                        // 減速
                        aDutyIndex -= indexDegree;
                    }
                    break;
                }
                case REVERSE: { // 逆方向

                    // インデックスが10以上の場合は一気に10減少させる
                    indexDegree = (aDutyIndex >= 10) ? 10 : 1;

                    // 減速
                    if(aDutyIndex > 0) {
                        aDutyIndex -= indexDegree;
                    }
                    // index=0 に達したのならBRAKE状態にする
                    if(aDutyIndex == 0) {
                        aDutyIndex = 0;
                        motorManager->changeMotorState(MOTOR_A, BRAKE);
                    }
                    break;
                }
                case STOP:
                case BRAKE:
                    // 正回転開始
                    motorManager->changeMotorState(MOTOR_A, FORWARD);
                    break;
                case UNKNOWN:
                    break;
            }

            // モータの回転方向に応じて処理分岐(MotorB)
            switch (bMotorDirection) {
                case FORWARD: { // 正方向（＝プローブの回転方向と等しい）

                    // 差が10以上離れている場合はインデックスを一気に10増加させる
                    indexDegree = (gz128 - bDutyIndex >= 10) ? 10 : 1;

                    if(gz128 > bDutyIndex) {
                        // 加速
                        bDutyIndex += indexDegree;
                    } else if (gz128 < bDutyIndex){
                        // 減速
                        bDutyIndex -= indexDegree;
                    }
                    break;
                }
                case REVERSE: { // 逆方向

                    // インデックスが10以上の場合は一気に10減少させる
                    indexDegree = (bDutyIndex >= 10) ? 10 : 1;

                    // 減速
                    if(bDutyIndex > 0) {
                        bDutyIndex -= indexDegree;
                    }
                    // index=0 に達したのならBRAKE状態にする
                    if(bDutyIndex == 0) {
                        bDutyIndex = 0;
                        motorManager->changeMotorState(MOTOR_B, BRAKE);
                    }
                    break;
                }
                case STOP:
                case BRAKE:
                    // 正回転開始
                    motorManager->changeMotorState(MOTOR_B, FORWARD);
                    break;
                case UNKNOWN:
                    break;
            }

        }
        /**
         * プローブは逆方向に回転
         */
        else if (gz128 < -1) { // -2 <-> -128

            // モータの回転方向に応じて処理分岐(MotorA)
            switch (aMotorDirection) {
                case REVERSE: { // 逆方向（＝プローブの回転方向と等しい）

                    uint8_t absGz128 = abs(gz128);

                    // 差が10以上離れている場合はインデックスを一気に10増加させる
                    indexDegree = (absGz128 - aDutyIndex >= 10) ? 10 : 1;

                    if(absGz128 > aDutyIndex) {
                        // 加速
                        aDutyIndex += indexDegree;
                    } else if (absGz128 < aDutyIndex){
                        // 減速
                        aDutyIndex -= indexDegree;
                    }
                    break;
                }
                case FORWARD: { // 正方向

                    // インデックスが10以上の場合は一気に10減少させる
                    indexDegree = (aDutyIndex >= 10) ? 10 : 1;

                    // 減速
                    if(aDutyIndex > 0) {
                        aDutyIndex -= indexDegree;
                    }
                    // index=0 に達したのならBRAKE状態にする
                    if(aDutyIndex == 0) {
                        aDutyIndex = 0;
                        motorManager->changeMotorState(MOTOR_A, BRAKE);
                    }
                    break;
                }
                case STOP:
                case BRAKE:
                    // 逆回転開始
                    motorManager->changeMotorState(MOTOR_A, REVERSE);
                    break;
                default:
                    break;
            }

            // モータの回転方向に応じて処理分岐(MotorB)
            switch (bMotorDirection) {
                case REVERSE: { // 逆方向（＝プローブの回転方向と等しい）

                    uint8_t absGz128 = abs(gz128);

                    // 差が10以上離れている場合はインデックスを一気に10増加させる
                    indexDegree = (absGz128 - bDutyIndex >= 10) ? 10 : 1;

                    if(absGz128 > bDutyIndex) {
                        // 加速
                        bDutyIndex += indexDegree;
                    } else if (absGz128 < bDutyIndex){
                        // 減速
                        bDutyIndex -= indexDegree;
                    }
                    break;
                }
                case FORWARD: { // 正方向

                    // インデックスが10以上の場合は一気に10減少させる
                    indexDegree = (bDutyIndex >= 10) ? 10 : 1;

                    // 減速
                    if(bDutyIndex > 0) {
                        bDutyIndex -= indexDegree;
                    }
                    // index=0 に達したのならBRAKE状態にする
                    if(bDutyIndex == 0) {
                        bDutyIndex = 0;
                        motorManager->changeMotorState(MOTOR_B, BRAKE);
                    }
                    break;
                }
                case STOP:
                case BRAKE:
                    // 逆回転開始
                    motorManager->changeMotorState(MOTOR_B, REVERSE);
                    break;
                default:
                    break;
            }
        // 平衡状態
        } else {
            if(aDutyIndex > 0) {
                aDutyIndex--; //MEMO: 高回転で平衡状態の場合、この処理が悪影響を及ぼす可能性がある
            }
            if(bDutyIndex > 0) {
                bDutyIndex--; //MEMO: 高回転で平衡状態の場合、この処理が悪影響を及ぼす可能性がある
            }
        }

//        DEBUG_PRINT("gz:%d, index:%d, duty:%d\r\n", gz128, aDutyIndex, (int)(dutyTable[indexReverse] * 100));
//        DEBUG_PRINT("raw:%d, 128:%d, idxA:%d idxB:%d\r\n", gz_raw, gz128, aDutyIndex, bDutyIndex);

        // dutyを更新する
        motorManager->aServo->write(dutyTableX2[aDutyIndex]);
        motorManager->bServo->write(dutyTableX2[bDutyIndex]);

        // 直近の値を保存しておく
        gz128Last = gz128;

        // ログに記録
        if(config->enableLogging) {
            logData(config->statusFlags, gz_raw, aDutyIndex, bDutyIndex, motorManager->aRPM, motorManager->bRPM);
        }
    }
}

/**
 * リカバリ不可能なエラーが発生した場合に無限ループでLEDを点滅させる
 */
static void indicateError() {
    while(true) {
        led->write(!(led->read()));
        wait(0.1);
    }
}

/**
 * TEST: 力率あたりの回転数を測定する
 **/
void dutyTest() {

    // 稼働中の場合は一旦停止
    stopFlywheel();

    float duty = 0.0f;

    // stop motors
    motorManager->aServo->write(0.0f);
    motorManager->bServo->write(0.0f);
    wait_ms(1000); // wait 1 sec

    for (uint8_t i=0; duty<=1.0; i++) { // 0.01-1.00
        motorManager->aServo->write(duty);
        motorManager->bServo->write(1.0-duty);
        wait_ms(500); // wait 1 sec

        for (uint8_t j=0; j<10; j++) {
            motorManager->read();
            // duty, aCounter, bCounter, aCount, bCount, aRPM, bRPM
            serial->printf("%d, %d, %d\r\n"
                    , (int)(duty * 100)
                    , (int)motorManager->aRPM, (int)motorManager->bRPM);

            wait_ms(200); // wait 0.2 sec
        }
        duty += 0.05; // increase duty + 0.1
    }

    // stop motors
    motorManager->aServo->write(0.0f);
    motorManager->bServo->write(0.0f);
}

/**
 * TEST: 最大速度からブレーキをかける
 */
void brakeTest() {

    // 稼働中の場合は一旦停止
    stopFlywheel();

    /**
     * MOTOR A
     */
    DEBUG_PRINT("Motor A Brake Test\r\n", NULL);
    // start motor for 2 sec
    motorManager->aServo->write(1.0f);
    wait_ms(2000);

    // brake !!!
    DEBUG_PRINT("BRAKE !!!\r\n", NULL);
    motorManager->aServo->write(0.0f);
    wait_ms(5000); // wait 3 sec

    /**
     * MOTOR B
     */
    DEBUG_PRINT("Motor B Brake Test\r\n", NULL);
    // start motor for 2 sec
    motorManager->bServo->write(1.0f);
    wait_ms(2000);

    // brake !!!
    DEBUG_PRINT("BRAKE !!!\r\n", NULL);
    motorManager->bServo->write(0.0f);
    wait_ms(5000);

    DEBUG_PRINT("END OF BRAKE TEST\r\n", NULL);
}

/**
 * Serial
 */
// Interrupt Routine to read in data from serial port
void interruptRx() {

    // Loop just in case more than one character is in UART's receive FIFO buffer
    // Stop if buffer full
    while ((serial->readable()) && (((rxInPointer + 1) % serialBufferSize) != rxOutPointer)) {
        rxBuffer[rxInPointer] = serial->getc();

        // Uncomment to Echo to USB serial to watch data flow
        //serial->putc(rxBuffer[rxInPointer]); // echo back
        rxInPointer = (rxInPointer + 1) % serialBufferSize;
    }
}

// Read a line from the large rx buffer from rx interrupt routine
void readLine() {

    int i;
    i = 0;
    // Start Critical Section - don't interrupt while changing global buffer variables
    NVIC_DisableIRQ(UART_IRQn);

    // Loop reading rx buffer characters until end of line character
    while ((i==0) || (rxLineBuffer[i-1] != '\r')) { // '\r' = 0x0d
        rxLineBuffer[i] = rxBuffer[rxOutPointer];
        i++;
        rxOutPointer = (rxOutPointer + 1) % serialBufferSize;
    }

    // End Critical Section
    NVIC_EnableIRQ(UART_IRQn);
    rxLineBuffer[i-1] = 0;
}

/**
 * Config
 */
void printConfig() {
    unsigned char charValue[sizeof(time_t)];
    uint8_t i;

    serial->putc(config->statusFlags); // ステータスフラグ

    memcpy(charValue, &config->gyroBiasRawX, sizeof(int16_t));  // Gyro Bias(X)
    for(i=0;i<sizeof(int16_t);i++) {
        serial->putc(charValue[i]);
    }
    memcpy(charValue, &config->gyroBiasRawY, sizeof(int16_t));  // Gyro Bias(Y)
    for(i=0;i<sizeof(int16_t);i++) {
        serial->putc(charValue[i]);
    }
    memcpy(charValue, &config->gyroBiasRawZ, sizeof(int16_t));  // Gyro Bias(Z)
    for(i=0;i<sizeof(int16_t);i++) {
        serial->putc(charValue[i]);
    }

    serial->putc(config->enableLogging);                    // enable/disable logging

    memcpy(charValue, &config->logPointer, sizeof(uint16_t));   // latest log pointer
    for(i=0;i<sizeof(uint16_t);i++) {
        serial->putc(charValue[i]);
    }

    memcpy(charValue, &config->logStartTime, sizeof(time_t));   // logging started time
    for(i=0;i<sizeof(time_t);i++) {
        serial->putc(charValue[i]);
    }
}

void printConfigReadable() {

    /**
     * 0x0000 statusFlags(uint8_t:1)
     * 0x0001-0x0004 gyroBiasX(float:4)
     * 0x0005-0x0008 gyroBiasX(float:4)
     * 0x0009-0x000C gyroBiasX(float:4)
     * 0x000D enableLogging(uint8_t:1)
     * 0x000E-0x000F logPointer(uint16_t:2)
     * 0x0010-0x0013 logStartTime(time_t:4)
     */

    // status flag
    char statusByte = config->statusFlags;
    serial->printf("Er xx xx xx xx xx Ac Id\r\n");
    serial->printf("-----------------------\r\n");
    serial->printf(" %d  %d  %d  %d  %d  %d  %d  %d\r\n"
            , ((statusByte & 0x80) ? 1 : 0)
            , ((statusByte & 0x40) ? 1 : 0)
            , ((statusByte & 0x20) ? 1 : 0)
            , ((statusByte & 0x10) ? 1 : 0)
            , ((statusByte & 0x08) ? 1 : 0)
            , ((statusByte & 0x04) ? 1 : 0)
            , ((statusByte & 0x02) ? 1 : 0)
            , ((statusByte & 0x01) ? 1 : 0)
    );
    serial->printf("Gyro BiasRaw(x y z): %05d %05d %05d\r\n", config->gyroBiasRawX, config->gyroBiasRawY, config->gyroBiasRawZ);
    serial->printf("Enable Logging: %d\r\n", config->enableLogging);
    serial->printf("Log pointer: 0x%04X\r\n", config->logPointer);
    serial->printf("Logged at: %d\r\n", (time_t) config->logStartTime);
}

void loadConfig() {

    char *buffer = new char[CONFIG_AREA_SIZE];

    sram->read(0x0000, (char*)buffer, CONFIG_AREA_SIZE); // read 0x0000-0x0020

    /**
     * 0x0000 statusFlags(uint8_t:1)
     * 0x0001-0x0002 gyroBiasX(int16_t:2)
     * 0x0003-0x0004 gyroBiasX(int16_t:2)
     * 0x0005-0x0006 gyroBiasX(int16_t:2)
     * 0x0007 enableLogging(uint8_t:1)
     * 0x0008-0x0009 logPointer(uint16_t:2)
     * 0x000A-0x000D logStartTime(time_t:4)
     */

    config->statusFlags   = (uint8_t) buffer[0];
    config->gyroBiasRawX  = (uint16_t) (buffer[2] << 8 | buffer[1]);
    config->gyroBiasRawY  = (uint16_t) (buffer[4] << 8 | buffer[3]);
    config->gyroBiasRawZ  = (uint16_t) (buffer[6] << 8 | buffer[5]);
    config->enableLogging = (uint8_t) buffer[7];
    config->logPointer    = (uint16_t) (buffer[9] << 8 | buffer[8]);
    config->logStartTime  = (time_t) (buffer[13] << 24 | buffer[12] << 16 | buffer[11] << 8 | buffer[10]);

    delete[] buffer;
}

void saveConfig() {

    char charValue[sizeof(time_t)];

    /**
     * 0x0000 statusFlags(uint8_t:1)
     * 0x0001-0x0002 gyroBiasX(int16_t:2)
     * 0x0003-0x0004 gyroBiasX(int16_t:2)
     * 0x0005-0x0006 gyroBiasX(int16_t:2)
     * 0x0007 enableLogging(uint8_t:1)
     * 0x0008-0x0009 logPointer(uint16_t:2)
     * 0x000A-0x000D logStartTime(time_t:4)
     */

    // status flags
    sram->write(0x0000, config->statusFlags);

    // Gyro Bias
    memcpy(charValue, &config->gyroBiasRawX, sizeof(uint16_t));
    sram->write(0x0001, charValue, sizeof(uint16_t));
    memcpy(charValue, &config->gyroBiasRawY, sizeof(uint16_t));
    sram->write(0x0003, charValue, sizeof(uint16_t));
    memcpy(charValue, &config->gyroBiasRawZ, sizeof(uint16_t));
    sram->write(0x0005, charValue, sizeof(uint16_t));

    // enable logging
    sram->write(0x0007, config->enableLogging);

    // log pointer
    memcpy(charValue, &config->logPointer, sizeof(uint16_t));
    sram->write(0x0008, charValue, sizeof(uint16_t));

    // log start time
    memcpy(charValue, &config->logStartTime, sizeof(time_t));
    sram->write(0x000A, charValue, sizeof(time_t));

    // save data onto EEPROM
    sram->callHardwareStore();
}

/**
 * 設定値を初期化して変数とSRAMにそれぞれ保存する
 * 0x0000 statusFlags(uint8_t:1)
 * 0x0001-0x0002 gyroBiasX(int16_t:2)
 * 0x0003-0x0004 gyroBiasX(int16_t:2)
 * 0x0005-0x0006 gyroBiasX(int16_t:2)
 * 0x0007 enableLogging(uint8_t:1)
 * 0x0008-0x0009 logPointer(uint16_t:2)
 * 0x000A-0x000D logStartTime(time_t:4)
 */
void resetConfig() {
    // init vars
    config->statusFlags = IDLE;
    config->gyroBiasRawX  = 0;
    config->gyroBiasRawY  = 0;
    config->gyroBiasRawZ  = 0;
    config->enableLogging = 0;
    config->logPointer = LOG_START_AT;
    config->logStartTime = time(NULL); // current time(timestamp)

    // save SRAM
    saveConfig();
}

void dumpMemory() {
    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    for(uint16_t address=0x0000; address<SRAM_MAX_SIZE; address+=bufferLength) {

        // Seq. Read
        sram->read(address, buffer, bufferLength);

        for(uint8_t i=0; i<bufferLength; i++) {
            serial->putc(buffer[i]);
        }
    }

    delete[] buffer;
}

void dumpMemoryReadable() {
    static const uint8_t bufferLength = 16;
    char *buffer = new char[bufferLength];

    serial->printf("ADDR 00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f\r\n");
    serial->printf("----------------------------------------------------\r\n");

    for(uint16_t address=0x0000; address<SRAM_MAX_SIZE; address+=bufferLength) {
        // Seq. Read
        sram->read(address, buffer, bufferLength);

        serial->printf("%04x ", address);
        for(uint8_t i=0; i<bufferLength; i++) {
            serial->printf("%02x ", buffer[i]);
        }
        serial->printf("\r\n");
    }

    delete[] buffer;
}

void clearLog(uint16_t startAddress, uint16_t endAddress){

    uint8_t retryCount = 0;
    for (uint16_t i=startAddress; i<endAddress; i++) {
        retryCount = 0;
        while(sram->write(i, 0x00) != 0 && retryCount++ < 5) {
            sram->write(i, 0x00);
        }
    }

    // save data onto EEPROM
    sram->callHardwareStore();
}

/**
 * save current data onto SRAM
 */
void logData(uint8_t _currentStatus, int16_t _gz_raw, uint8_t _aDutyIndex, uint8_t _bDutyIndex, uint16_t _aRPM, uint16_t _bRPM) {

    // if overflow, reset pointer
    if(config->logPointer + CONFIG_LOG_SIZE > SRAM_MAX_SIZE) {
        config->logPointer = LOG_START_AT;
    }

    // write current status
    sram->write(config->logPointer++, _currentStatus);

    // write gz_raw
    sram->write(config->logPointer, _gz_raw);
    config->logPointer += 2;

    // write duty index for Motor A
    sram->write(config->logPointer++, _aDutyIndex);

    // write duty index for Motor B
    sram->write(config->logPointer++, _bDutyIndex);

    // write RPM for Motor A
    sram->write(config->logPointer, _aRPM);
    config->logPointer += 2;

    // write RPM for Motor B
    sram->write(config->logPointer, _bRPM);
    config->logPointer += 2;

    // save current logging pointer
    sram->write(0x0008, config->logPointer);
}
