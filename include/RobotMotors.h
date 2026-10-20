#pragma once
#include <Arduino.h>
#include "RobotConfig.h"

class RobotMotors {
private:
    const int pwmFreq = 5000;      
    const int pwmResolution = 8;   
    const int pwmChannelLeft = 0;  
    const int pwmChannelRight = 1; 

public:
    void begin() {
        pinMode(MOTOR_AIN1, OUTPUT);
        pinMode(MOTOR_AIN2, OUTPUT);
        pinMode(MOTOR_BIN1, OUTPUT);
        pinMode(MOTOR_BIN2, OUTPUT);

        #if ESP_IDF_VERSION_MAJOR < 5
            ledcSetup(pwmChannelLeft, pwmFreq, pwmResolution);
            ledcAttachPin(MOTOR_PWMA, pwmChannelLeft);
            
            ledcSetup(pwmChannelRight, pwmFreq, pwmResolution);
            ledcAttachPin(MOTOR_PWMB, pwmChannelRight);
        #else
            ledcAttachChannel(MOTOR_PWMA, pwmFreq, pwmResolution, pwmChannelLeft);
            ledcAttachChannel(MOTOR_PWMB, pwmFreq, pwmResolution, pwmChannelRight);
        #endif
        
        stop(); 
    }

    // Điều khiển bánh trái (GIỮ NGUYÊN)
    void setLeftMotor(int speed) {
        if (speed > 0) { // Lệnh Tiến -> Đảo HIGH/LOW để ép động cơ quay tới
            digitalWrite(MOTOR_AIN1, LOW);
            digitalWrite(MOTOR_AIN2, HIGH);
        } else if (speed < 0) { // Lệnh Lùi
            digitalWrite(MOTOR_AIN1, HIGH);
            digitalWrite(MOTOR_AIN2, LOW);
        } else { // Dừng
            digitalWrite(MOTOR_AIN1, LOW);
            digitalWrite(MOTOR_AIN2, LOW);
        }
        
        #if ESP_IDF_VERSION_MAJOR < 5
            ledcWrite(pwmChannelLeft, abs(speed));
        #else
            ledcWriteChannel(pwmChannelLeft, abs(speed));
        #endif
    }

    // Điều khiển bánh phải (ĐÃ ĐẢO CHIỀU LOGIC)
    void setRightMotor(int speed) {
        if (speed > 0) { // Lệnh Tiến -> Ép động cơ phải quay ngược lại để xe đi thẳng
            digitalWrite(MOTOR_BIN1, LOW);
            digitalWrite(MOTOR_BIN2, HIGH);
        } else if (speed < 0) { // Lệnh Lùi -> Ép quay chiều kia
            digitalWrite(MOTOR_BIN1, HIGH);
            digitalWrite(MOTOR_BIN2, LOW);
        } else { // Dừng
            digitalWrite(MOTOR_BIN1, LOW);
            digitalWrite(MOTOR_BIN2, LOW);
        }
        
        #if ESP_IDF_VERSION_MAJOR < 5
            ledcWrite(pwmChannelRight, abs(speed));
        #else
            ledcWriteChannel(pwmChannelRight, abs(speed));
        #endif
    }

    void setSpeeds(int speedLeft, int speedRight) {
        speedLeft = constrain(speedLeft, -255, 255);
        speedRight = constrain(speedRight, -255, 255);
        
        setLeftMotor(speedLeft);
        setRightMotor(speedRight);
    }

    void stop() {
        setSpeeds(0, 0);
    }
};