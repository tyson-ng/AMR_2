#pragma once
#include <Arduino.h>

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
        integral = 0.0; last_error = 0.0;
    }

    float compute(float setpoint, float measured_value, float dt) {
        if (dt <= 0.0) return 0.0;

        float error = setpoint - measured_value;
        integral += error * dt;
        
        // Anti-windup
        if (ki != 0) {
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
    
    void reset() { integral = 0.0; last_error = 0.0; }
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
        integral = 0.0;
    }

    float compute(float target_yaw, float current_yaw, float current_gyro_z, float dt) {
        if (dt <= 0.0) return 0.0;

        // BƯỚC 1: Tính và chuẩn hóa sai số góc [-180, 180] chống quay vòng tròn
        float error_yaw = target_yaw - current_yaw;
        if (error_yaw > 180.0) error_yaw -= 360.0;
        if (error_yaw < -180.0) error_yaw += 360.0;

        // BƯỚC 2: Tính khâu P và I
        float p_out = kp * error_yaw;
        
        integral += ki * error_yaw * dt;
        // Anti-windup cho góc
        if (ki != 0) {
            float max_integral = max_delta_v / ki;
            if (integral > max_integral) integral = max_integral;
            else if (integral < -max_integral) integral = -max_integral;
        }

        // BƯỚC 3: Dùng Gyro Z thế vào khâu Đạo hàm (D) để triệt tiêu nhiễu
        float d_out = kd * (0.0 - current_gyro_z);

        // BƯỚC 4: Tổng hợp Delta V và cắt xén để chống lật xe
        float delta_v = p_out + integral + d_out;
        
        if (delta_v > max_delta_v) delta_v = max_delta_v;
        else if (delta_v < -max_delta_v) delta_v = -max_delta_v;

        return delta_v;
    }

    void reset() { integral = 0.0; }
};