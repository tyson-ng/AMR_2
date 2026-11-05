#pragma once
#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotEncoder.h"
#include "RobotImu.h"
#include "RobotMotors.h"
#include "RobotPID.h"
#include "RobotMicroROS.h"

#if APP_MODE == 2
#include "RobotBluetooth.h"
#endif

class RobotTester {
private:
    float base_velocity = 0.0f;   // Vận tốc dài đặt (m/s)
    float target_yaw = 0.0f;      // Góc hướng đặt (độ)
    unsigned long last_time_us = 0;

    float computeDt() {
        unsigned long now_us = micros();
        if (last_time_us == 0) {
            last_time_us = now_us;
            return 0.02f;
        }
        float dt = (now_us - last_time_us) / 1000000.0f;
        last_time_us = now_us;
        if (dt <= 0.0001f || dt > 0.2f) dt = 0.02f;
        return dt;
    }

public:
    #if APP_MODE == 2
    void updateBluetooth(RobotBluetooth &bt,
                         RobotEncoder &leftEnc,
                         RobotEncoder &rightEnc,
                         RobotImu &imu,
                         RobotMotors &motors,
                         RobotPID &pidLeft,
                         RobotPID &pidRight,
                         HeadingPID &pidHeading) {
        float dt = computeDt();
        imu.update();

        if (bt.hasData()) {
            String cmd = bt.readStringUntil('\n');
            cmd.trim();
            if (cmd.startsWith("V") || cmd.startsWith("v")) {
                base_velocity = cmd.substring(1).toFloat();
                target_yaw = imu.getYaw();
                bt.print("Set Velocity: ");
                bt.println(base_velocity, 3);
            } else if (cmd.startsWith("Y") || cmd.startsWith("y")) {
                target_yaw = cmd.substring(1).toFloat();
                bt.print("Set Target Yaw: ");
                bt.println(target_yaw, 1);
            } else if (cmd.startsWith("S") || cmd.startsWith("s")) {
                base_velocity = 0.0f;
                pidLeft.reset();
                pidRight.reset();
                pidHeading.reset();
                motors.stop();
                bt.println("STOPPED");
            } else if (cmd.startsWith("R") || cmd.startsWith("r")) {
                leftEnc.reset();
                rightEnc.reset();
                imu.reset();
                target_yaw = 0.0f;
                bt.println("RESET OK");
            }
        }

        float v_L = leftEnc.getVelocity(dt);
        float v_R = rightEnc.getVelocity(dt);

        float delta_v = 0.0f;
        if (fabsf(base_velocity) > 0.001f || fabsf(HeadingPID::normalizeAngleDeg(target_yaw - imu.getYaw())) > 1.0f) {
            delta_v = pidHeading.compute(target_yaw, imu.getYaw(), imu.getGyroZ(), dt);
        }

        float v_target_L = base_velocity - delta_v;
        float v_target_R = base_velocity + delta_v;

        int pwm_L = (int)pidLeft.compute(v_target_L, v_L, dt);
        int pwm_R = (int)pidRight.compute(v_target_R, v_R, dt);

        motors.setSpeeds(pwm_L, pwm_R);

        static int print_div = 0;
        if (++print_div >= 10) {
            print_div = 0;
            bt.print("Yaw:"); bt.print(imu.getYaw(), 1);
            bt.print(" | vL:"); bt.print(v_L, 2);
            bt.print(" | vR:"); bt.println(v_R, 2);
        }
    }
    #endif

    void updateROS(RobotEncoder &leftEnc,
                   RobotEncoder &rightEnc,
                   RobotImu &imu,
                   RobotMotors &motors,
                   RobotPID &pidLeft,
                   RobotPID &pidRight,
                   HeadingPID &pidHeading,
                   RobotMicroROS &ros_node) {
        float dt = computeDt();

        // 1. Cập nhật IMU & nhận gói tin /cmd_vel từ ROS 2 Agent
        imu.update();
        ros_node.spinSome();

        // 2. Đo vận tốc thực tế, góc bánh xe và tính động học thuận
        float v_L = leftEnc.getVelocity(dt);
        float v_R = rightEnc.getVelocity(dt);
        float pos_L_rad = leftEnc.getPositionRad();
        float pos_R_rad = rightEnc.getPositionRad();
        float vel_L_rad = v_L / WHEEL_RADIUS_M;
        float vel_R_rad = v_R / WHEEL_RADIUS_M;

        float v_x_measured = (v_R + v_L) * 0.5f;
        float v_theta_measured = (v_R - v_L) / WHEEL_BASE_M;

        // 3. Lấy vận tốc mục tiêu từ ROS 2 (/cmd_vel)
        float cmd_vx = 0.0f, cmd_wz = 0.0f;
        ros_node.getCmdVel(cmd_vx, cmd_wz);

        float v_target_L = 0.0f;
        float v_target_R = 0.0f;

        if (fabsf(cmd_vx) < 0.001f && fabsf(cmd_wz) < 0.001f) {
            pidLeft.reset();
            pidRight.reset();
            pidHeading.reset();
            motors.stop();
            target_yaw = imu.getYaw();
        } else if (fabsf(cmd_wz) < 0.001f) {
            // Đi thẳng: Dùng HeadingPID khóa góc hướng chống lệch quỹ đạo
            float delta_v = pidHeading.compute(target_yaw, imu.getYaw(), imu.getGyroZ(), dt);
            v_target_L = cmd_vx - delta_v;
            v_target_R = cmd_vx + delta_v;

            int pwm_L = (int)pidLeft.compute(v_target_L, v_L, dt);
            int pwm_R = (int)pidRight.compute(v_target_R, v_R, dt);
            motors.setSpeeds(pwm_L, pwm_R);
        } else {
            // Đang xoay theo lệnh /cmd_vel từ Teleop hoặc Nav2
            target_yaw = imu.getYaw();
            pidHeading.reset();

            v_target_L = cmd_vx - (cmd_wz * WHEEL_BASE_M * 0.5f);
            v_target_R = cmd_vx + (cmd_wz * WHEEL_BASE_M * 0.5f);

            int pwm_L = (int)pidLeft.compute(v_target_L, v_L, dt);
            int pwm_R = (int)pidRight.compute(v_target_R, v_R, dt);
            motors.setSpeeds(pwm_L, pwm_R);
        }

        // 4. Xuất bản /telemetry (KaiaaiTelemetry2) & /imu/data ở tần số 50Hz
        ros_node.publishData(
            v_x_measured,
            v_theta_measured,
            imu.getYaw(),
            imu.getGyroZ(),
            pos_L_rad,
            pos_R_rad,
            vel_L_rad,
            vel_R_rad,
            dt);
    }
};