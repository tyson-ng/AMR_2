#pragma once

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
const float WHEEL_DIAMETER_M = 0.044; 
const float PI_VAL = 3.14159265;
const float TICKS_PER_REVOLUTION = 1400.0;

// --- CHÂN CẮM YDLIDAR X3 ---
#define LIDAR_RX 16   // Nối vào TX của Lidar
#define LIDAR_TX 12   // Nối vào RX của Lidar
#define LIDAR_EN 13   // Chân Enable (nếu có)
#define LIDAR_PWM 17  // Chân điều khiển tốc độ quay (nếu có)