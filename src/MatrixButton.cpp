#include "MatrixButton.h"

// ピン定義
#define MT_OUT_PIN1 D10 // Matrix pin
#define MT_OUT_PIN2 D9  // Matrix pin 2
#define MT_OUT_PIN3 D8  // Matrix pin 3
#define MT_IN_PIN1 D7   // Matrix pin 4
#define MT_IN_PIN2 D6   // Matrix pin 5
#define MT_IN_PIN3 D3   // Matrix pin 6

/*
 * ボタンマトリックス割り当て表
 * 
 * OUTx + INy の組み合わせで対応する入力を定義
 * 
 * ┌────────┬─────────────────┬─────────────────┬───────────────┐
 * │        │    IN1          │    IN2          │    IN3        │
 * ├────────┼─────────────────┼─────────────────┼───────────────┤
 * │ OUT1   │ マウス左クリック │ キーボードJキー  │ マウス戻るボタン│
 * │ OUT2   │ キーボードIキー  │ ENTERキー       │ キーボードKキー │
 * │ OUT3   │ マウス右クリック │ キーボードLキー  │ マウス進むボタン│
 * └────────┴─────────────────┴────────────────┴────────────────┘
 */

MatrixButton::MatrixButton() {
    // 出力ピン配列の初期化
    mt_out_pins[0] = MT_OUT_PIN1;
    mt_out_pins[1] = MT_OUT_PIN2;
    mt_out_pins[2] = MT_OUT_PIN3;
    
    // 入力ピン配列の初期化
    mt_in_pins[0] = MT_IN_PIN1;
    mt_in_pins[1] = MT_IN_PIN2;
    mt_in_pins[2] = MT_IN_PIN3;
    
    // ボタン状態の初期化
    for (int i = 0; i < OUT_PIN_COUNT; i++) {
        for (int j = 0; j < IN_PIN_COUNT; j++) {
            button_states[i][j] = false;
        }
    }
}

void MatrixButton::begin() {
    initializeMatrixPins();
}

void MatrixButton::initializeMatrixPins() {
    // 出力ピンの設定
    for (int i = 0; i < OUT_PIN_COUNT; i++) {
        pinMode(mt_out_pins[i], OUTPUT);
        digitalWrite(mt_out_pins[i], HIGH);  // 初期状態はHIGH
    }
    
    // 入力ピンの設定
    for (int i = 0; i < IN_PIN_COUNT; i++) {
        pinMode(mt_in_pins[i], INPUT_PULLUP);
    }
}

void MatrixButton::scan() {
    // マトリックスのスキャン
    for (int out = 0; out < OUT_PIN_COUNT; out++) {
        // 現在の出力ピンをLOWにして有効化
        digitalWrite(mt_out_pins[out], LOW);
        
        // 少し待ってピンの状態が安定するのを待つ
        delayMicroseconds(10);
        
        // 入力ピンの状態を読み取る
        for (int in = 0; in < IN_PIN_COUNT; in++) {
            // ボタンが押されたらLOW
            bool state = (digitalRead(mt_in_pins[in]) == LOW);
            button_states[out][in] = state;
        }
        
        // 出力ピンを元に戻す
        digitalWrite(mt_out_pins[out], HIGH);
    }
}

bool MatrixButton::isPressed(int outIndex, int inIndex) {
    if (outIndex >= 0 && outIndex < OUT_PIN_COUNT && 
        inIndex >= 0 && inIndex < IN_PIN_COUNT) {
        return button_states[outIndex][inIndex];
    }
    return false;
}
