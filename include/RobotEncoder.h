#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "RobotConfig.h"

class RobotEncoder {
private:
    ESP32Encoder encoder;
    bool is_reversed = false;
    int64_t prev_ticks = 0;

public:
    void begin(int pinA, int pinB, bool reverse = false) {
        ESP32Encoder::useInternalWeakPullResistors = puType::up;
        encoder.attachFullQuad(pinA, pinB);
        encoder.clearCount();
        is_reversed = reverse;
        prev_ticks = 0;
    }

    int64_t getTicks() {
        int64_t ticks = encoder.getCount();
        return is_reversed ? -ticks : ticks;
    }

    float getDistance() {
        int64_t ticks = getTicks();
        return ((float)ticks / TICKS_PER_REVOLUTION) * (PI_VAL * WHEEL_DIAMETER_M);
    }

    // Góc quay tích lũy của bánh xe (rad) phục vụ KaiaaiTelemetry2 joint_pos
    float getPositionRad() {
        int64_t ticks = getTicks();
        return ((float)ticks / TICKS_PER_REVOLUTION) * (2.0f * PI_VAL);
    }

    // Vận tốc dài tức thời của bánh xe (m/s)
    float getVelocity(float dt) {
        if (dt <= 0.0f) return 0.0f;
        int64_t current_ticks = getTicks();
        int64_t delta_ticks = current_ticks - prev_ticks;
        prev_ticks = current_ticks;

        float delta_dist = ((float)delta_ticks / TICKS_PER_REVOLUTION) * (PI_VAL * WHEEL_DIAMETER_M);
        return delta_dist / dt;
    }

    void reset() {
        encoder.clearCount();
        prev_ticks = 0;
    }
};