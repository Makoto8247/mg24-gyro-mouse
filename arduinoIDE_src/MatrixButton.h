#ifndef MATRIX_BUTTON_H
#define MATRIX_BUTTON_H

#include <Arduino.h>

class MatrixButton {
public:
    MatrixButton();
    void begin();
    void scan();
    bool isPressed(int outIndex, int inIndex);

private:
    static const int OUT_PIN_COUNT = 3;
    static const int IN_PIN_COUNT = 3;
    
    int mt_out_pins[OUT_PIN_COUNT];
    int mt_in_pins[IN_PIN_COUNT];
    bool button_states[OUT_PIN_COUNT][IN_PIN_COUNT];
    
    void initializeMatrixPins();
};

#endif // MATRIX_BUTTON_H