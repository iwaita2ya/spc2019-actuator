#define DEBUG

#include <mbed.h>
#include "GsLSM9DS1.h"
#include "SerialSRAM.h"
#include "sineTable.h"
#include "SensorManager.h"
#include "MotorManager.h"


// sensor update frequency
#define UPDATE_SENSOR_FREQ 0.1f // センサ更新間隔(秒) //TODO: 本番運用時 <0.1

using namespace greysound;

/**
 * params
 */
static uint8_t aDutyIndex = 0;
static uint8_t indexReverse = 0;

/**
 * flags
 */
uint8_t isActive;

Ticker *sensorTicker;   // 9軸センサ更新タイマ
//SPI spi(dp2, dp1, dp6); // mosi, miso, sclk
RawSerial *serial; // tx, rx

#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) serial->printf(fmt, __VA_ARGS__)
#define DEBUG_PUTC(x) serial->putc(x)
#else
#define DEBUG_PRINT(x)
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

using namespace greysound;

SensorManager *sensorManager;
MotorManager *motorManager;

// Function Prototypes --------------------------------------------------------

// ９軸センサ更新
static void stopSensor();
static void startSensor();
static void updateSensor();

// エラー表示
static void indicateError();

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

    // set active flag
    isActive = 1;

    /**
     * Init Serial
     */
    serial = new RawSerial(P0_19, P0_18); // tx, rx
    serial->baud(115200); // default:9600bps

    // init SensorManager (and Ticker)
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

    /**
     * Init LED
     */
    led = new DigitalOut(P0_7);
    led->write(0); // set led off

    //-------------------------------------
    // Main
    //-------------------------------------

    // start motor
    DEBUG_PRINT("Start Motor\r\n", NULL);
    motorManager->start();

    //MEMO: TEST TEST TEST !!!
//    DEBUG_PRINT("Start DUTY TEST\r\n");
//    dutyTest(); //MEMO: DEBUG ONLY
//    DEBUG_PRINT("Start Brake Test\r\n");
//    brakeTest();

    // start sensor
    DEBUG_PRINT("Start Sensor\r\n", NULL);
    startSensor();


    DEBUG_PRINT("Start Main Loop\r\n", NULL);
    while(isActive == 1) {

//        motorManager->read();

        // RPM を表示
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
    stopSensor();
    delete(sensorManager);
    delete(sensorTicker);
    delete(sram);
    delete(serial);

    return 0;
}

/**
 * Functions
 */

static void startSensor()
{
    // センサ開始
    if(sensorManager && sensorManager->getCurrentState() == SensorManager::STAND_BY)
    {
        // failed to start
        if (sensorManager->begin() == 0) {
            indicateError();
            return; //TODO: エラーからリカバリできるようにする
        };
    }

    // センサ値更新処理開始
    if(sensorTicker) {
        sensorTicker->attach(&updateSensor, UPDATE_SENSOR_FREQ);
    }
}


static void stopSensor()
{
    // センサ値更新処理停止
    if(sensorTicker) {
        sensorTicker->detach();
    }

    // センサ停止
    if(sensorManager && sensorManager->getCurrentState() != SensorManager::STAND_BY) {
        sensorManager->end();
    }
}

/**
 * センサの値に応じてサーボの回転方向とDuty比を調整する
 */
static void updateSensor()
{
    static int16_t gz_raw = 0;
    static int8_t gz128=0, gz128Last=0;
    static uint8_t indexDegree=1;
    motorState aMotorDirection;

    if(sensorManager) {

        // read sensor values
        sensorManager->read();

        // get gyro z-axis rotation value
        gz_raw = sensorManager->lsm9dof->gz_raw; // -32768<->32767

        // MEMO: 256(-128<->127)段階に変換
        gz128 = uint8_t ((gz_raw >> 8) * 0.8 + gz128Last * 0.3); // -128<->127

        // モータの回転状態を取得(FORWARD|REVERSE|STOP|BRAKE)
        aMotorDirection = motorManager->getMotorState(MOTOR_A);

        // プローブは正方向に回転
        if(gz128 > 0) {

            // モータの回転方向に応じて処理分岐
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

        }
        // プローブは逆方向に回転
        else if (gz128 < -1) { // -2 <-> -128

            // モータの回転方向に応じて処理分岐
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

        } else {
            // 平衡状態
            if(aDutyIndex > 0) {
                aDutyIndex--; //MEMO: 高回転で平衡状態の場合、この処理が悪影響を及ぼす可能性がある
            }
        }

        DEBUG_PRINT("raw:%d, 128:%d, index:%d, duty(x100):%d\r\n", gz_raw, gz128, aDutyIndex, (int)(dutyTable[indexReverse] * 100));
//        DEBUG_PRINT("gz:%d, index:%d, duty:%d\r\n", gz128, aDutyIndex, (int)(dutyTable[indexReverse] * 100));

        // dutyを更新する
        motorManager->aServo->write(dutyTable[aDutyIndex]);
        motorManager->bServo->write(dutyTable[indexReverse]);

        // 直近の値を保存しておく
        gz128Last = gz128;
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