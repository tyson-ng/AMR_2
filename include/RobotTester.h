#pragma once
#include <Arduino.h>
#include "RobotBluetooth.h"
#include "RobotEncoder.h"
#include "RobotImu.h"
#include "RobotMotors.h"
#include "RobotPID.h" // Chứa cả RobotPID và HeadingPID

class RobotTester {
private:
    int testMode;
    unsigned long last_bt_time;   
    unsigned long last_motor_time; 
    
    float last_dist_L, last_dist_R;
    int current_pwm;
    
    float base_vel;         
    float target_vel_L, target_vel_R; 
    float target_yaw;       
    bool is_open_loop; 

    void processCommand(String cmd, RobotBluetooth &bt, RobotEncoder &leftMotor, RobotEncoder &rightMotor, 
                        RobotImu &imu, RobotMotors &motors, RobotPID &pidLeft, RobotPID &pidRight, HeadingPID &pidHeading) {
        cmd.trim(); 
        
        if (cmd == "1") testMode = 1;
        else if (cmd == "2") testMode = 2;
        else if (cmd == "3") testMode = 3;
        else if (cmd == "0") {
            leftMotor.reset(); rightMotor.reset(); imu.reset();
            bt.println(">> DA RESET TOA DO <<");
        }
        else if (cmd.startsWith("P")) {
            current_pwm = cmd.substring(1).toInt();
            motors.setSpeeds(current_pwm, current_pwm);
            is_open_loop = true;
            base_vel = 0; target_vel_L = 0; target_vel_R = 0; 
            if (current_pwm != 0) {
                Serial.println("Time(ms),PWM,Vel_Left(m/s),Vel_Right(m/s)");
                bt.println("Time(ms),PWM,Vel_Left(m/s),Vel_Right(m/s)");
            } else {
                Serial.println(">> DA DUNG DONG CO <<");
            }
        }
        else if (cmd.startsWith("V")) {
            float speed = cmd.substring(1).toFloat();
            speed = constrain(speed, -0.4, 0.4);

            base_vel = speed; 
            target_yaw = imu.getYaw(); // Chốt góc đi thẳng
            
            is_open_loop = false;
            current_pwm = 0; 
            
            Serial.print(">> Da bat Cascade PID. Van toc: "); Serial.print(speed); Serial.println(" m/s <<");
        }
        else if (cmd == "STOP") {
            base_vel = 0; target_vel_L = 0; target_vel_R = 0; current_pwm = 0;
            motors.stop(); 
            pidLeft.reset(); pidRight.reset(); pidHeading.reset();
            Serial.println(">> DA DUNG XE <<");
        }
    }

public:
    RobotTester() {
        testMode = 3;
        last_bt_time = millis();
        last_motor_time = millis();
        last_dist_L = 0; last_dist_R = 0;
        current_pwm = 0; base_vel = 0; 
        target_vel_L = 0; target_vel_R = 0; target_yaw = 0; 
        is_open_loop = true; 
    }

    // NHẬN THÊM ĐỐI TƯỢNG pidHeading VÀO ĐÂY
    void update(RobotBluetooth &bt, RobotEncoder &leftMotor, RobotEncoder &rightMotor, 
                RobotImu &imu, RobotMotors &motors, RobotPID &pidLeft, RobotPID &pidRight, HeadingPID &pidHeading) {
        
        imu.update();
        unsigned long current_time = millis();
        
        if (bt.hasData()) {
            String btCmd = bt.readStringUntil('\n');
            processCommand(btCmd, bt, leftMotor, rightMotor, imu, motors, pidLeft, pidRight, pidHeading);
        }
        if (Serial.available()) {
            String serialCmd = Serial.readStringUntil('\n');
            processCommand(serialCmd, bt, leftMotor, rightMotor, imu, motors, pidLeft, pidRight, pidHeading);
        }

        float dt = (current_time - last_motor_time) / 1000.0;
        if (dt >= 0.02) {
            float dist_L = leftMotor.getDistance();
            float dist_R = rightMotor.getDistance();
            float vel_L = (dist_L - last_dist_L) / dt;
            float vel_R = (dist_R - last_dist_R) / dt;
            
            last_dist_L = dist_L; last_dist_R = dist_R;
            last_motor_time = current_time;

            if (is_open_loop) {
                if (current_pwm != 0) {
                    Serial.print(current_time); Serial.print(","); Serial.print(current_pwm); Serial.print(",");
                    Serial.print(vel_L, 4); Serial.print(","); Serial.println(vel_R, 4);
                }
            } else {
                // ========================================================
                // CHẠY CASCADE VÀ MIXER TRỘN VẬN TỐC
                // ========================================================
                if (base_vel != 0) {
                    // 1. Chạy Vòng ngoài lấy Delta_V
                    float current_yaw = imu.getYaw();
                    float current_gyro_z = imu.getGyroZ();
                    float Delta_V = pidHeading.compute(target_yaw, current_yaw, current_gyro_z, dt);

                    // 2. Mixer chia vận tốc
                    target_vel_L = base_vel - Delta_V;
                    target_vel_R = base_vel + Delta_V;
                } else {
                    target_vel_L = 0; target_vel_R = 0;
                }

                // 3. Chạy Vòng trong
                int pwm_L = (int)pidLeft.compute(target_vel_L, vel_L, dt);
                int pwm_R = (int)pidRight.compute(target_vel_R, vel_R, dt);
                
                if (base_vel != 0) {
                    motors.setSpeeds(pwm_L, pwm_R);
                    
                    static int pid_print_count = 0;
                    if (pid_print_count++ % 10 == 0) { 
                        bt.print("Yaw Err: "); bt.print(target_yaw - imu.getYaw(), 2);
                        bt.print(" | Set L: "); bt.print(target_vel_L, 2); 
                        bt.print(" | Set R: "); bt.println(target_vel_R, 2); 
                    }
                } else {
                    motors.stop();
                }
            }
        }

        if (current_time - last_bt_time >= 200) {
            last_bt_time = current_time; 
            if (current_pwm == 0 && base_vel == 0) {
                if (testMode == 1 || testMode == 3) {
                    bt.print("[ENCODER] L: "); bt.print(leftMotor.getDistance(), 3); 
                    bt.print("m | R: "); bt.print(rightMotor.getDistance(), 3); bt.println("m");
                }
                if (testMode == 2 || testMode == 3) {
                    if (imu.isReady()) {
                        bt.print("[IMU] Yaw: "); bt.println(imu.getYaw(), 2);
                    }
                }
            }
        }
    }
};