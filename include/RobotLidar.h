#pragma once
#include <Arduino.h>
#include "lds_all_models.h"
#include "RobotConfig.h"

// Khởi tạo các đối tượng toàn cục (Bắt buộc do thư viện LDS dùng C-style Callback)
HardwareSerial LidarSerial(2); // Dùng UART2 (Chân 16, 17)
LDS_YDLIDAR_X3_PRO lidar;

// --- CÁC HÀM GIAO TIẾP VÀ XỬ LÝ (CALLBACKS) ---

int lidar_serial_read_callback() {
    return LidarSerial.read();
}

size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
    return LidarSerial.write(buffer, length);
}

// Hàm này sẽ tự động chạy mỗi khi Lidar quét được 1 điểm ảnh mới
void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
    // Tạm thời chỉ in ra màn hình Serial máy tính để test (không gửi qua Bluetooth cho đỡ lag)
    // Cứ 40 điểm in 1 lần để bạn đọc kịp
    static int count = 0;
    if (count++ % 40 == 0) {
        Serial.print("[LIDAR] Goc: ");
        Serial.print(angle_deg, 1);
        Serial.print(" do | Cach: ");
        Serial.print(distance_mm, 1);
        Serial.println(" mm");
    }
}

// Hàm này tự động băm xung PWM để Lidar quay ổn định
void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
    int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_EN : LIDAR_PWM;

    if (value <= (float)LDS::DIR_INPUT) {
        if (value == (float)LDS::DIR_OUTPUT_PWM) {
            #if ESP_IDF_VERSION_MAJOR < 5
            ledcSetup(2, 10000, 11); // Kênh 2, 10kHz, 11-bit
            ledcAttachPin(pin, 2);
            #else
            ledcAttachChannel(pin, 10000, 11, 2);
            #endif
        } else {
            pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
        }
        return;
    }

    if (value < (float)LDS::VALUE_PWM) {
        digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
    } else {
        int pwm_value = ((1<<11)-1)*value;
        #if ESP_IDF_VERSION_MAJOR < 5
        ledcWrite(2, pwm_value);
        #else
        ledcWriteChannel(2, pwm_value);
        #endif
    }
}

// --- CLASS ĐÓNG GÓI CHO HỆ THỐNG OOP ---
class RobotLidar {
public:
    void begin() {
        // Gắn các hàm xử lý vào thư viện
        lidar.setScanPointCallback(lidar_scan_point_callback);
        lidar.setSerialWriteCallback(lidar_serial_write_callback);
        lidar.setSerialReadCallback(lidar_serial_read_callback);
        lidar.setMotorPinCallback(lidar_motor_pin_callback);

        // Mở cổng Serial2 với tốc độ Baud mặc định của X3 (thường là 115200)
        LidarSerial.begin(lidar.getSerialBaudRate(), SERIAL_8N1, LIDAR_RX, LIDAR_TX);

        lidar.init();
        LDS::result_t result = lidar.start();
        
        Serial.print("Trang thai khoi dong Lidar: ");
        Serial.println(lidar.resultCodeToString(result));
    }

    void update() {
        lidar.loop();
    }
};