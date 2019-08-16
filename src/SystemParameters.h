//
// Created by Tatsuya Iwai (GreySound) on 19/06/17.
//

#ifndef SPC2019_ACTUATOR_SYSTEM_PARAMETERS_H
#define SPC2019_ACTUATOR_SYSTEM_PARAMETERS_H

/**
 * 0x0000 statusFlags(1)
 * 0x0001-0x0004 gyroBiasX(4)
 * 0x0005-0x0008 gyroBiasX(4)
 * 0x0009-0x000C gyroBiasX(4)
 * 0x000D enableLogging(1)
 * 0x000E-0x0011 logStartTime(4)
 * 0x0012-0x0013 logPointer(2)
 */
struct SystemParameters {
    uint8_t statusFlags;
    float gyroBiasX;        // ジャイロの補正値(X軸)
    float gyroBiasY;        // ジャイロの補正値(Y軸)
    float gyroBiasZ;        // ジャイロの補正値(Z軸)
    uint8_t enableLogging;  // ロギング設定 (0x00:無効 0x01:有効)
    uint16_t logPointer;    // 最終ログ格納アドレス (0x0020-0x0800)
    time_t logStartTime;    // ロギング開始時刻(RTCから取得する）
};

#endif //SPC2019_ACTUATOR_SYSTEM_PARAMETERS_H
