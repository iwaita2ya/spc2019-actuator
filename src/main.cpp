#define DEBUG

#include <mbed.h>
#include "GsLSM9DS1.h"
#include "Gs47SerialSRAM.h"
#include "sineTable.h"
#include "SensorManager.h"
#include "MotorManager.h"


// sensor update frequency
#define UPDATE_SENSOR_FREQ 0.1f // センサ更新間隔(秒)

/**
 * params
 */
static uint8_t indexForward = 0;
static uint8_t indexReverse = 0;

/**
 * flags
 */
uint8_t isActive;

Ticker *sensorTicker;   // 9軸センサ更新タイマ
//SPI spi(dp2, dp1, dp6); // mosi, miso, sclk
RawSerial *serial; // tx, rx

#ifdef DEBUG
#define DEBUG_PRINT(x) serial->printf(x)
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
Gs47SerialSRAM *sram;

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
    DEBUG_PRINT("Init SensorManager\r\n");
    sensorTicker = new Ticker();
    sensorManager = new SensorManager(P0_5, P0_4, 0xD6, 0x3C); // sda, scl, agAddr, mAddr, LED1
    sensorManager->init();

    /**
     * Init MotorManager
     * forward, reverse, standBy, aIn1, aIn2, bIn1, bIn2, aCount, bCount
     */
    DEBUG_PRINT("Init MotorManager\r\n");
    motorManager = new MotorManager(P0_9, P0_10, P0_8, P0_11, P0_12, P0_13, P0_14, P0_15, P0_16);

    /**
     * init SRAM
     */
    DEBUG_PRINT("Init SRAM\r\n");
    sram = new Gs47SerialSRAM(P0_5, P0_4, P0_6); // sda, scl, hs, A2=0, A1=0

    /**
     * Init LED
     */
    led = new DigitalOut(P0_7);
    led->write(0); // set led off

    //-------------------------------------
    // Main
    //-------------------------------------

    // start sensor
    DEBUG_PRINT("Start Sensor\r\n");
    //startSensor();

    DEBUG_PRINT("Start Sensor\r\n");
    // start motor
    motorManager->start();
    motorManager->aServo->write(0.5); // duty 50%
    motorManager->bServo->write(0.5); // duty 50%

    DEBUG_PRINT("Start Main Loop\r\n");
    while(isActive == 1) {

        motorManager->read();
        serial->printf("duration: %lu aCounter:%ld bCounter:%ld aCount:%ld bCount:%ld aRPM: %d bRPM %d\r\n"
                , motorManager->duration
                , motorManager->aCounter, motorManager->bCounter
                , motorManager->aCount, motorManager->bCount
                , (int)motorManager->aRPM, (int)motorManager->bRPM);

        led->write(!(led->read()));
        wait(0.5);
    }

    // stop & terminate objects
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

    if(sensorManager) {

        // read sensor values
        sensorManager->read();

        // get gyro z-axis rotation value
        gz_raw = sensorManager->lsm9dof->gz_raw; // 0x0000-0xFFFF

        // 値が小さい(<0x00FF)場合は正転・逆転共に減速
        if(abs(gz_raw) < 255) {
            if(indexForward > 0) {
                indexForward--;
            }
            if(indexReverse > 0) {
                indexReverse--;
            }
        }
        // 回転方向に応じて加速・減速する
        else if(gz_raw > 0) {
            // forward++　正転加速・逆転減速
            if (indexForward < TABLE_INDEX) {
                indexForward++;
            }
            if (indexReverse > 0) {
                indexReverse--;
            }
        } else {
            // reverse++　正転減速・逆転加速
            if (indexForward > 0) {
                indexForward--;
            }
            if (indexReverse < TABLE_INDEX) {
                indexReverse++;
            }
        }

        //MEMO: DEBUG ONLY
        DEBUG_PUTC(((gz_raw >> 8) & 0xFF));  // gz high
        DEBUG_PUTC(gz_raw & 0xFF);         // gz low
        DEBUG_PUTC(indexForward);
        DEBUG_PUTC(indexReverse);

        // dutyを更新する
//        servoForward.write(sineTable[indexForward]);
//        servoReverse.write(sineTable[indexReverse]);
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

