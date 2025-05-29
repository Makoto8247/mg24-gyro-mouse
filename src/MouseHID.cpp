#include "MouseHID.h"

MouseHID::MouseHID(uint8_t* reportArray,
                   float sensitivity,
                   float deadZone,
                   float threshold) {
    this->reportArray = reportArray;
    this->mouseSensitivity = sensitivity;
    this->mouseDeadZone = deadZone;
    this->mouseThreshold = threshold;

    currentMouseData = {.delta_x = 0, 
                        .delta_y = 0, 
                        .buttons = 0};
}

void MouseHID::processAcceleration(float accelX, float accelY) {
    // デッドゾーン処理
    if (fabs(accelX) < mouseDeadZone) accelX = 0;
    if (fabs(accelY) < mouseDeadZone) accelY = 0;

    // 感度調整
    accelX *= mouseSensitivity;
    accelY *= mouseSensitivity;

    // 範囲制限
    if (accelX > mouseThreshold) {
        currentMouseData.delta_x = mouseThreshold; // 最大値
    } else if (accelX < -mouseThreshold) {
        currentMouseData.delta_x = -mouseThreshold; // 最小値
    } else {
        currentMouseData.delta_x = (int8_t)accelX; // 範囲内の値
    }

    if (accelY > mouseThreshold) {
        currentMouseData.delta_y = mouseThreshold; // 最大値
    } else if (accelY < -mouseThreshold) {
        currentMouseData.delta_y = -mouseThreshold; // 最小値
    } else {
        currentMouseData.delta_y = (int8_t)accelY; // 範囲内の値
    }

    updateReport();
}

void MouseHID::setButton(uint8_t button, bool pressed) {
    if (pressed) {
        currentMouseData.buttons |= button; // ボタンを押す
    } else {
        currentMouseData.buttons &= ~button; // ボタンを離す
    }
    updateReport();
}

void MouseHID::setButtons(uint8_t buttonMask) {
    currentMouseData.buttons = buttonMask;
    updateReport();
}

mouse_data MouseHID::getMouseData() {
    return currentMouseData;
}

void MouseHID::updateReport() {
    reportArray[0] = currentMouseData.delta_x;
    reportArray[1] = currentMouseData.delta_y;
    reportArray[2] = currentMouseData.buttons;
}
