#pragma once
#include <Arduino.h>
#include "lds_all_models.h"
#include "RobotConfig.h"

class RobotLidar {
private:
    static const size_t RING_BUFFER_SIZE = 2048;
    static uint8_t rx_ring_buf[RING_BUFFER_SIZE];
    static volatile size_t head;
    static volatile size_t tail;
    static volatile size_t count;
    static volatile bool scan_completed_flag;
    static portMUX_TYPE buf_mux;

    static LDS_YDLIDAR_X3_PRO& getLidar() {
        static LDS_YDLIDAR_X3_PRO lidar;
        return lidar;
    }

    static int lidar_serial_read_callback() {
        return getSerial().read();
    }

    static size_t lidar_serial_write_callback(const uint8_t * buffer, size_t length) {
        return getSerial().write(buffer, length);
    }

    static void lidar_scan_point_callback(float angle_deg, float distance_mm, float quality, bool scan_completed) {
        if (scan_completed) {
            scan_completed_flag = true;
        }
    }

    static void lidar_motor_pin_callback(float value, LDS::lds_pin_t lidar_pin) {
        int pin = (lidar_pin == LDS::LDS_MOTOR_EN_PIN) ? LIDAR_EN : LIDAR_PWM;
        if (value <= (float)LDS::DIR_INPUT) {
            pinMode(pin, (value == (float)LDS::DIR_INPUT) ? INPUT : OUTPUT);
            return;
        }
        if (value < (float)LDS::VALUE_PWM) {
            digitalWrite(pin, (value == (float)LDS::VALUE_HIGH) ? HIGH : LOW);
        }
    }

public:
    static HardwareSerial& getSerial() {
        static HardwareSerial LidarSerial(2);
        return LidarSerial;
    }

    void begin() {
        LDS_YDLIDAR_X3_PRO& lidar = getLidar();
        HardwareSerial& LidarSerial = getSerial();

        lidar.setScanPointCallback(lidar_scan_point_callback);
        lidar.setSerialWriteCallback(lidar_serial_write_callback);
        lidar.setSerialReadCallback(lidar_serial_read_callback);
        lidar.setMotorPinCallback(lidar_motor_pin_callback);

        LidarSerial.setRxBufferSize(2048);
        LidarSerial.begin(lidar.getSerialBaudRate(), SERIAL_8N1, LIDAR_RX, LIDAR_TX);

        lidar.init();
        lidar.start();
        Serial.println("[LIDAR] Khoi dong xong phan cung!");
    }

    // Đẩy trực tiếp 1 byte thô từ UART vào Ring Buffer
    static void pushByte(uint8_t b) {
        portENTER_CRITICAL(&buf_mux);
        if (count < RING_BUFFER_SIZE) {
            rx_ring_buf[head] = b;
            head = (head + 1) % RING_BUFFER_SIZE;
            count++;
        }
        portEXIT_CRITICAL(&buf_mux);
    }

    // Rút mảng byte thô sang gói KaiaaiTelemetry2
    static size_t popLdsData(uint8_t* dest, size_t max_len, bool &scan_start_hint) {
        size_t bytes_to_read = 0;
        portENTER_CRITICAL(&buf_mux);
        scan_start_hint = scan_completed_flag;
        scan_completed_flag = false;

        bytes_to_read = (count < max_len) ? count : max_len;
        for (size_t i = 0; i < bytes_to_read; i++) {
            dest[i] = rx_ring_buf[tail];
            tail = (tail + 1) % RING_BUFFER_SIZE;
        }
        count -= bytes_to_read;
        portEXIT_CRITICAL(&buf_mux);
        return bytes_to_read;
    }

    static size_t availableBytes() {
        size_t avail = 0;
        portENTER_CRITICAL(&buf_mux);
        avail = count;
        portEXIT_CRITICAL(&buf_mux);
        return avail;
    }
};

uint8_t RobotLidar::rx_ring_buf[RobotLidar::RING_BUFFER_SIZE];
volatile size_t RobotLidar::head = 0;
volatile size_t RobotLidar::tail = 0;
volatile size_t RobotLidar::count = 0;
volatile bool RobotLidar::scan_completed_flag = false;
portMUX_TYPE RobotLidar::buf_mux = portMUX_INITIALIZER_UNLOCKED;