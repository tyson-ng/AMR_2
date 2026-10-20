#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "RobotConfig.h"

class RobotEncoder {
private:
    ESP32Encoder encoder;
    bool is_reversed; // Biến nhớ trạng thái đảo chiều

public:
    // Thêm tham số reverse (mặc định là false nếu không điền)
    void begin(int pinA, int pinB, bool reverse = false) {
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        encoder.attachFullQuad(pinA, pinB);
        encoder.clearCount();
        is_reversed = reverse;
    }

    // Trả về quãng đường đã được xử lý dấu
    float getDistance() {
        long ticks = encoder.getCount();
        float distance = (ticks / TICKS_PER_REVOLUTION) * (PI_VAL * WHEEL_DIAMETER_M);
        return is_reversed ? -distance : distance; // Đảo dấu nếu bị ngược
    }

    void reset() {
        encoder.clearCount();
    }
};