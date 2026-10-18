#pragma once
#include <Arduino.h>
#include "RobotBluetooth.h"
#include "RobotEncoder.h"
#include "RobotImu.h"

class RobotTester {
private:
    int testMode;

public:
    RobotTester() {
        testMode = 3;
    }

    // Hàm cập nhật (Nhận cả đối tượng Bluetooth vào)
    void update(RobotBluetooth &bt, RobotEncoder &leftMotor, RobotEncoder &rightMotor, RobotImu &imu) {
        
        // 1. Nhận lệnh từ điện thoại qua Bluetooth
        if (bt.hasData()) {
            char c = bt.readCommand();
            if (c == '1') testMode = 1;
            else if (c == '2') testMode = 2;
            else if (c == '3') testMode = 3;
            else if (c == '0') {
                leftMotor.reset();
                rightMotor.reset();
                imu.reset();
                bt.println(">> DA RESET TOA DO <<");
            }
        }

        // 2. In dữ liệu Encoder
        if (testMode == 1 || testMode == 3) {
            bt.print("[ENCODER] Trai: ");
            bt.print(leftMotor.getDistance(), 3); 
            bt.print(" m | Phai: ");
            bt.print(rightMotor.getDistance(), 3);
            bt.println(" m");
        }

        // 3. In dữ liệu IMU
        if (testMode == 2 || testMode == 3) {
            if (imu.isReady()) {
                bt.print("[IMU] Goc Yaw: ");
                bt.print(imu.getYaw(), 2);
                bt.println(" do");
            } else {
                bt.println("[IMU] Loi ket noi I2C!");
            }
        }
        
        if (testMode == 3) bt.println("--------------------");
    }
};