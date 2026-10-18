#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "RobotConfig.h"

class RobotImu {
private:
    float yawAngle;
    unsigned long lastImuTime;
    float gyroZ_offset;
    bool ready;

public:
    RobotImu() {
        yawAngle = 0.0;
        gyroZ_offset = 0.0;
        ready = false;
    }

    // Ép chân I2C và đánh thức IMU
    void begin() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x6B); // Thanh ghi PWR_MGMT_1
        Wire.write(0);    // Ghi 0 để đánh thức
        
        if (Wire.endTransmission(true) == 0) {
            ready = true;
            // Dải đo 500 độ/s
            Wire.beginTransmission(MPU_ADDR);
            Wire.write(0x1B);
            Wire.write(0x08);
            Wire.endTransmission(true);
            lastImuTime = millis();
        } else {
            ready = false;
        }
    }

    // Hàm này sẽ được gọi liên tục trong Task 100Hz
    void update() {
        if (!ready) return;
        
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47); 
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)2, true);
        
        if (Wire.available() == 2) {
            int16_t gyroZ_raw = Wire.read() << 8 | Wire.read();
            float gyroZ_deg = ((float)gyroZ_raw / 65.5) - gyroZ_offset;
            
            unsigned long currentTime = millis();
            float dt = (currentTime - lastImuTime) / 1000.0;
            lastImuTime = currentTime;

            if (abs(gyroZ_deg) > 1.0) {
                yawAngle += gyroZ_deg * dt; 
            }
        }
    }

    float getYaw() { return yawAngle; }
    bool isReady() { return ready; }
    void reset() { yawAngle = 0.0; }
};