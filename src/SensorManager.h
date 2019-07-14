//
// Created by iwait on 7/14/19.
//

#ifndef SPC2019_ACTUATOR_SENSORMANAGER_H
#define SPC2019_ACTUATOR_SENSORMANAGER_H

#include <mbed.h>
#include "GsLSM9DS1.h"
#include "Gs47SerialSRAM.h"
#include "sineTable.h"

namespace greysound {

// センサ管理クラス
    class SensorManager {

    public:
        GsLSM9DS1 *lsm9dof;
        int activeTimeMs;

        // センサのステータス
        enum SensorState {
            CREATED,
            STAND_BY,
            ACTIVE,
            BUSY,
            BEGIN_FAILED
        };

        /**
         * コンストラクタ
         * @param _sda
         * @param _scl
         * @param _agAddr
         * @param _mAddr
         */
        SensorManager(PinName _sda, PinName _scl, uint8_t _agAddr, uint8_t _mAddr)
        {
            lsm9dof = new GsLSM9DS1(_sda, _scl, _agAddr, _mAddr);
            activeTimeMs = 0;
            currentState = CREATED;
        }

        ~SensorManager(){
            // stop all activities
            end();

            // メモリ解放
            delete lsm9dof;
        }

        void init()
        {
            // init sensor
            lsm9dof->init();

            // perform calibration
            lsm9dof->calibrate();
            lsm9dof->calibrateMag(true); // true: save calculated bias onto register

            currentState = STAND_BY;
        }

        uint8_t begin()
        {
            // prepare sensor starting
            switch (currentState) {
                case CREATED:
                case BEGIN_FAILED:
                    this->init();
                    break;

                case ACTIVE:
                case BUSY:
                    this->end();
                    break;

                case STAND_BY:
                default:
                    break;
            }

            // start sensor update
            // Make sure communication is working
            if (lsm9dof->begin() == 0) {
                currentState = BEGIN_FAILED;
                return 0;
            }

            // reset & start timer
            activeTimer.reset();
            activeTimer.start();
            activeTimeMs = 0;

            currentState = ACTIVE;
            return 1;
        }

        // データを読み取る
        uint8_t read()
        {
            // すでに処理が走っている場合は中止
            if (currentState == BUSY) {
                return 0;
            }

            currentState = BUSY;

            // current time (ms)
            activeTimeMs = activeTimer.read_ms();

            // 9dof
//        lsm9dof->readAccel();
            lsm9dof->readGyro();
//        lsm9dof->readMag();
            //lsm9dof->readTemp();

            currentState = ACTIVE;

            return 1;
        }

        /**
         * ステータスに関係なく強制的にセンサの値を更新する
         * (速度が要求されない待機状態での使用を想定)
         */
        void forceRead()
        {
            // 9dof
            lsm9dof->readAccel();
            lsm9dof->readGyro();
            lsm9dof->readMag();
            //lsm9dof->readTemp();
        }

        void end()
        {
            activeTimer.stop();
            currentState = STAND_BY;
        }

        SensorState getCurrentState()
        {
            return currentState;
        }

    protected:

    private:
        SensorState currentState;
        Timer activeTimer;
    };
}
#endif //SPC2019_ACTUATOR_SENSORMANAGER_H
