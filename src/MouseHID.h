#ifndef MOUSE_HID_H
#define MOUSE_HID_H

#include <Arduino.h>

#define ACCEL_THRESHOLD 15.0f
#define DEAD_ZONE       2.0f
#define SENSITIVITY     1.2f

#define MOUSE_LEFT_BUTTON   0x01
#define MOUSE_RIGHT_BUTTON  0x02
#define MOUSE_MIDDLE_BUTTON 0x04

struct mouse_data {
    int8_t delta_x;
    int8_t delta_y;
    uint8_t buttons;
};

struct mouse_accel_data {
    int32_t accel_x;
    int32_t accel_y;
};

class MouseHID {
public:
    MouseHID(uint8_t* reportArray,
             float sensitivity = SENSITIVITY, 
             float deadZone = DEAD_ZONE, 
             float threshold = ACCEL_THRESHOLD);

    // 加速度からマウス移動量を計算
    void processAcceleration(float accel_x, float accel_y);

    // マウスのボタン状態を更新
    void setButton(uint8_t button, bool pressed);
    void setButtons(uint8_t buttonMask);

    // マウスデータを取得
    mouse_data getMouseData();

private:
    float mouseSensitivity;
    float mouseDeadZone;
    float mouseThreshold;
    mouse_data currentMouseData;
    uint8_t* reportArray;
    void updateReport();
};

#endif
