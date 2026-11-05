#pragma once

// --- CẤU HÌNH XUẤT BẢN ODOMETRY ---
// 0: Chuẩn Kaia.ai (ESP32 phát /telemetry + /imu/data, node kaiaai_telemetry trên PC sẽ phát /odom, /scan, /tf)
// 1: ESP32 tự phát thêm topic /odom trực tiếp (Dùng khi test độc lập không bật kaiaai_telemetry)
#define ENABLE_DIRECT_ODOM_PUB 0

// --- CHÂN CẮM ENCODER ---
#define ENCA_A 18
#define ENCA_B 19
#define ENCB_A 4
#define ENCB_B 5

// --- THÔNG SỐ I2C (IMU) ---
const int MPU_ADDR = 0x68;
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;

// --- THÔNG SỐ VẬT LÝ CỦA XE ---
const float WHEEL_DIAMETER_M = 0.044f; 
const float WHEEL_RADIUS_M = WHEEL_DIAMETER_M * 0.5f;
const float PI_VAL = 3.14159265f;
const float TICKS_PER_REVOLUTION = 1400.0f;
const float WHEEL_BASE_M = 0.15f;

// --- CHÂN CẮM YDLIDAR X3 ---
#define LIDAR_RX 16   // Nối vào TX của Lidar
#define LIDAR_TX 12   // Nối vào RX của Lidar
#define LIDAR_EN 13   // Chân Enable (nếu có)
#define LIDAR_PWM 17  // Chân điều khiển tốc độ quay 

// --- CHÂN CẮM ĐỘNG CƠ ---
#define MOTOR_PWMA 32
#define MOTOR_AIN2 33
#define MOTOR_AIN1 25
#define MOTOR_BIN1 26
#define MOTOR_BIN2 27
#define MOTOR_PWMB 14