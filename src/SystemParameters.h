//
// Created by Tatsuya Iwai (GreySound) on 19/06/17.
//

#ifndef SPC2019_ACTUATOR_SYSTEM_PARAMETERS_H
#define SPC2019_ACTUATOR_SYSTEM_PARAMETERS_H

/**
 * 0x0000 statusFlags(uint8_t:1)
 * 0x0001-0x0002 gyroBiasX(int16_t:2)
 * 0x0003-0x0004 gyroBiasX(int16_t:2)
 * 0x0005-0x0006 gyroBiasX(int16_t:2)
 * 0x0007 enableLogging(uint8_t:1)
 * 0x0008-0x0009 logPointer(uint16_t:2)
 * 0x000A-0x000D logStartTime(time_t:4)
 */
struct SystemParameters {
    uint8_t statusFlags;
    int16_t gyroBiasRawX;   // ジャイロの補正値(X軸)
    int16_t gyroBiasRawY;   // ジャイロの補正値(Y軸)
    int16_t gyroBiasRawZ;   // ジャイロの補正値(Z軸)
    uint8_t enableLogging;  // ロギング設定 (0x00:無効 0x01:有効)
    uint16_t logPointer;    // 最終ログ格納アドレス (0x0020-0x0800)
    time_t logStartTime;    // ロギング開始時刻(RTCから取得する）
};

#endif //SPC2019_ACTUATOR_SYSTEM_PARAMETERS_H
