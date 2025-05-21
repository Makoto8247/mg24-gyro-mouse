#ifndef MOUSE_HID_H
#define MOUSE_HID_H

#include <Arduino.h>

struct mouse_data {
    int8_t delta_x;
    int8_t delta_y;
    uint8_t buttons;
};

struct mouse_accel_data {
    int32_t accel_x;
    int32_t accel_y;
};

#endif