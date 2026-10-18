#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "RobotConfig.h"

class RobotEncoder {
private:
    ESP32Encoder encoder;

public:
    // Khởi tạo chân và bộ đếm
    void begin(int pinA, int pinB) {
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        encoder.attachFullQuad(pinA, pinB);
        encoder.clearCount();
    }

    // Trả về thẳng quãng đường đã đi (Mét)
    float getDistance() {
        long ticks = encoder.getCount();
        return (ticks / TICKS_PER_REVOLUTION) * (PI_VAL * WHEEL_DIAMETER_M);
    }

    // Reset bộ đếm về 0
    void reset() {
        encoder.clearCount();
    }
};