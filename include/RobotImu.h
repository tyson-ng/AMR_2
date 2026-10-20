#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "RobotConfig.h"

class RobotImu {
private:
    float yawAngle;
    float current_gyro_z;
    unsigned long lastImuTime;
    float gyroZ_offset;
    bool ready;

public:
    RobotImu() {
        yawAngle = 0.0;
        current_gyro_z = 0.0;
        gyroZ_offset = 0.0; 
        ready = false;
    }

    void begin() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        Wire.setClock(400000); 

        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B); 
        Wire.write(0x00);    
        
        if (Wire.endTransmission() == 0) {
            delay(100); // Chờ chip khởi động phần cứng
            ready = true;
            
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(0x1B);
            Wire.write(0x08);
            Wire.endTransmission();
            
            // =======================================================
            // AUTO CALIBRATION (TIÊU CHUẨN CÔNG NGHIỆP)
            // =======================================================
            Serial.println("IMU: Dang hieu chuan Gyro... Vui long giu xe DUNG IM!");
            
            // 1. Chạy rà trơn (Warm-up): Vứt bỏ 100 mẫu rác đầu tiên
            for (int i = 0; i < 100; i++) {
                Wire.beginTransmission(MPU_ADDR);
                Wire.write(0x47); 
                Wire.endTransmission(); 
                Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)2);
                if (Wire.available() >= 2) {
                    Wire.read(); Wire.read();
                }
                delay(2);
            }

            // 2. Thu thập 500 mẫu chuẩn
            long total_raw = 0;
            int samples = 500;
            for (int i = 0; i < samples; i++) {
                Wire.beginTransmission(MPU_ADDR);
                Wire.write(0x47); 
                Wire.endTransmission();
                Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)2);
                
                if (Wire.available() >= 2) {
                    // ÉP KIỂU INT16_T: Chống lỗi đọc sai số âm (Trị dứt điểm góc -20000)
                    int16_t raw = (Wire.read() << 8) | Wire.read();
                    total_raw += raw;
                }
                delay(2);
            }
            
            // 3. Tính toán độ lệch gốc
            gyroZ_offset = ((float)total_raw / samples) / 65.5;
            
            Serial.print("IMU: Hieu chuan xong! Sai so bu tru = ");
            Serial.println(gyroZ_offset);
            // =======================================================

            lastImuTime = micros(); 
        } else {
            ready = false;
            Serial.println("IMU: LOI I2C! Kiem tra lai day dien.");
        }
    }

    void update() {
        if (!ready) return;
        
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47); 
        Wire.endTransmission(); 
        Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)2);
        
        if (Wire.available() >= 2) {
            // Phải ép đúng kiểu int16_t
            int16_t gyroZ_raw = (Wire.read() << 8) | Wire.read();
            
            float gyroZ_deg = ((float)gyroZ_raw / 65.5) - gyroZ_offset;
            
            unsigned long currentTime = micros();
            float dt = (currentTime - lastImuTime) / 1000000.0; 
            lastImuTime = currentTime;

            // Bộ lọc vi rung: Bỏ qua các tín hiệu nhiễu dưới 0.15 độ/s
            if (abs(gyroZ_deg) < 0.15) {
                gyroZ_deg = 0.0;
            }

            current_gyro_z = gyroZ_deg; 
            yawAngle += gyroZ_deg * dt; 
        }
    }

    float getYaw() { return yawAngle; }
    float getGyroZ() { return current_gyro_z; }
    bool isReady() { return ready; }
    
    void reset() { 
        yawAngle = 0.0; 
        current_gyro_z = 0.0; 
    }
};