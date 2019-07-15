//
// Created by iwait on 7/14/19.
//

#ifndef SPC2019_ACTUATOR_MOTORMANAGER_H
#define SPC2019_ACTUATOR_MOTORMANAGER_H

#include <mbed.h>

// motor duty update frequency
#define PERIOD_MIN 0.00005f //  20KHz(0.05ms)
#define PERIOD_MAX 0.00001f // 100KHz(0.05ms)

extern RawSerial *serial;

namespace greysound {

    class MotorManager {

    public:

        PwmOut *aServo; // forward PWM
        PwmOut *bServo; // reverse PWM
        DigitalOut *standBy; // /Stand-By
        DigitalOut *aIn1, *aIn2, *bIn1, *bIn2;
        InterruptIn *aInterrupt, *bInterrupt;

        uint32_t aCounter, bCounter;
        uint32_t timerValue, duration;
        uint32_t aCount, bCount;
        uint32_t aLastCount, bLastCount;
        float aRPM, bRPM, aLastRPM, bLastRPM;
        float aDuty, bDuty;

        // センサの状態管理
        enum MotorState {
            CREATED,        // インスタンス生成完了
            STAND_BY,       // データ取得準備完了
            ACTIVE,         // データ取得中
            BUSY,           // データ更新中
            BEGIN_FAILED    // データ取得開始失敗
        };

        MotorManager(PinName _forward, PinName _reverse, PinName _standBy, PinName _aIn1, PinName _aIn2, PinName _bIn1, PinName _bIn2, PinName _aCount, PinName _bCount)
        {
            // create instance
            aServo = new PwmOut(_forward);
            bServo = new PwmOut(_reverse);

            // set period
            aServo->period(PERIOD_MIN); // 20KHz
            bServo->period(PERIOD_MIN); // 20KHz

            // set duty
            aDuty = 0.5f; // 50%
            bDuty = 0.5f; // 50%
            aServo->write(aDuty); // 0.0-1.0f
            bServo->write(bDuty); // 0.0-1.0f

            // set standby
            standBy = new DigitalOut(_standBy); // _StandBy
            standBy->write(0); // set as standby mode //FIXME:これだと電源喪失復帰時時NG

            // CW: IN1:High IN2:Low
            aIn1 = new DigitalOut(_aIn1);   // AIN1
            aIn2 = new DigitalOut(_aIn2);   // AIN2
            aIn1->write(1);
            aIn2->write(0);

            // CCW: IN1:Low IN2:High
            bIn1 = new DigitalOut(_bIn1);   // BIN1
            bIn2 = new DigitalOut(_bIn2);   // BIN2
            bIn1->write(1);
            bIn2->write(0);

            // Init Interrupts
            aInterrupt = new InterruptIn(_aCount); // CW Counter
            bInterrupt = new InterruptIn(_bCount); // CCW Counter

            // counters
            aCounter = 0;
            bCounter = 0;

            // counter values
            aCount = 0;
            bCount = 0;
            aLastCount = 0;
            bLastCount = 0;

            // timer
            activeTimer = new Timer();

            // timer values
            timerValue = 0;
            lastTimerValue = 0;
            duration = 0; // timerValue - lastTimerValue;

            // PRMs
            aRPM = 0.0f;
            bRPM = 0.0f;
            aLastRPM = 0.0f;
            bLastRPM = 0.0f;

            // CREATED 状態に遷移
            currentState = CREATED;
        }

        ~MotorManager()
        {
            aInterrupt->rise(NULL);
            bInterrupt->rise(NULL);

            delete(aServo);
            delete(bServo);
            delete(standBy);
            delete(aIn1);
            delete(aIn2);
            delete(bIn1);
            delete(bIn2);
            delete(aInterrupt);
            delete(bInterrupt);
        }

        void start() {
            // init vars
            reset();

            // タイマー開始
            activeTimer->start();
            currentState = ACTIVE;

            // enable interrupt
            aInterrupt->rise(callback(this, &MotorManager::countForward));
            bInterrupt->rise(callback(this, &MotorManager::countReverse));

            // disable standby
            standBy->write(1);
        }

        void read() {

            if(currentState != BUSY) {

                currentState = BUSY;

                // calc duration
                timerValue = activeTimer->read_ms(); // read
                duration = timerValue - lastTimerValue;

                // get counter value (increment)
                uint32_t aCountTemp = aCounter;
                uint32_t bCountTemp = bCounter;

                aCount = aCountTemp - aLastCount;
                bCount = bCountTemp - bLastCount;

                // k=0.8
                aRPM =  ((aCount * 1000 / duration) * 0.8) + (aLastRPM * 0.2);
                bRPM =  ((bCount * 1000 / duration) * 0.8) + (bLastRPM * 0.2);

                // save current values as last values
                lastTimerValue = timerValue;
                aLastCount = aCountTemp;
                bLastCount = bCountTemp;

                aLastRPM = aRPM;
                bLastRPM = bRPM;

                currentState = ACTIVE;
            }
        }

        void stop() {
            standBy->write(0); // disable PWMs

            // タイマー停止
            activeTimer->stop();

            // disable interrupt
            aInterrupt->rise(NULL);
            bInterrupt->rise(NULL);

            currentState = STAND_BY;
        }

        void reset() {
            activeTimer->reset();

            timerValue = 0;
            lastTimerValue = 0;
            duration = 0;

            aCounter = 0;
            bCounter = 0;
            aCount = 0;
            bCount = 0;
            aLastCount = 0;
            bLastCount = 0;

            aRPM = 0;
            bRPM = 0;
            aLastRPM = 0;
            bLastRPM = 0;
        }

    protected:
    private:

        MotorState currentState;
        Timer *activeTimer;
        uint32_t lastTimerValue;

        void countForward() {
            aCounter++;
        }

        void countReverse() {
            bCounter++;
        }

    };
}
#endif //SPC2019_ACTUATOR_MOTORMANAGER_H
