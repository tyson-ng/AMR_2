#pragma once
#include <Arduino.h>
#include <math.h>

// ==========================================================
// 1. PID VÒNG TRONG (INNER LOOP - ĐIỀU KHIỂN VẬN TỐC BÁNH XE)
// ==========================================================
class RobotPID {
private:
    float kp, ki, kd;
    float integral, last_error;
    float output_min, output_max;

public:
    RobotPID(float p, float i, float d, float min_val, float max_val) {
        kp = p; ki = i; kd = d;
        output_min = min_val; output_max = max_val;
        integral = 0.0f; last_error = 0.0f;
    }

    float compute(float setpoint, float measured_value, float dt) {
        if (dt <= 0.0f) return 0.0f;

        // Xóa tích phân khi yêu cầu dừng hẳn và bánh đã dừng để tránh rè động cơ
        if (fabsf(setpoint) < 0.001f && fabsf(measured_value) < 0.005f) {
            integral = 0.0f;
            last_error = 0.0f;
            return 0.0f;
        }

        float error = setpoint - measured_value;
        integral += error * dt;
        
        if (ki != 0.0f) {
            float max_integral = output_max / ki;
            float min_integral = output_min / ki;
            if (integral > max_integral) integral = max_integral;
            else if (integral < min_integral) integral = min_integral;
        }

        float derivative = (error - last_error) / dt;
        float output = (kp * error) + (ki * integral) + (kd * derivative);
        last_error = error;

        if (output > output_max) output = output_max;
        else if (output < output_min) output = output_min;

        return output;
    }
    
    void setTunings(float p, float i, float d) { kp = p; ki = i; kd = d; }
    void reset() { integral = 0.0f; last_error = 0.0f; }
};

// ==========================================================
// 2. PID VÒNG NGOÀI (OUTER LOOP - ĐIỀU KHIỂN GÓC HƯỚNG)
// ==========================================================
class HeadingPID {
private:
    float kp, ki, kd;
    float integral;
    float max_delta_v;

public:
    HeadingPID(float p, float i, float d, float max_v) {
        kp = p; ki = i; kd = d;
        max_delta_v = max_v;
        integral = 0.0f;
    }

    // Chuẩn hóa mọi góc bất kỳ về đoạn [-180, 180]
    static float normalizeAngleDeg(float angle) {
        angle = fmodf(angle + 180.0f, 360.0f);
        if (angle < 0.0f) angle += 360.0f;
        return angle - 180.0f;
    }

    float compute(float target_yaw, float current_yaw, float current_gyro_z, float dt) {
        if (dt <= 0.0f) return 0.0f;

        // BƯỚC 1: Chuẩn hóa sai số góc về [-180, 180] bất kể xe quay bao nhiêu vòng
        float error_yaw = normalizeAngleDeg(target_yaw - current_yaw);

        // BƯỚC 2: Tính khâu P và I
        float p_out = kp * error_yaw;
        
        integral += ki * error_yaw * dt;
        if (ki != 0.0f) {
            if (integral > max_delta_v) integral = max_delta_v;
            else if (integral < -max_delta_v) integral = -max_delta_v;
        }

        // BƯỚC 3: Dùng Gyro Z thế vào khâu Đạo hàm (D) để triệt tiêu Derivative Kick
        float d_out = -kd * current_gyro_z;

        // BƯỚC 4: Tổng hợp Delta V và giới hạn
        float delta_v = p_out + integral + d_out;
        
        if (delta_v > max_delta_v) delta_v = max_delta_v;
        else if (delta_v < -max_delta_v) delta_v = -max_delta_v;

        return delta_v;
    }

    void setTunings(float p, float i, float d) { kp = p; ki = i; kd = d; }
    void reset() { integral = 0.0f; }
};