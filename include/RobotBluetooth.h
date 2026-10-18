#pragma once
#include <Arduino.h>
#include "BluetoothSerial.h"

class RobotBluetooth {
private:
    BluetoothSerial bt;

public:
    // Khởi động Bluetooth
    void begin(const String& name) {
        bt.begin(name);
    }

    // Kiểm tra có dữ liệu đến không
    bool hasData() {
        return bt.available() > 0;
    }

    // Đọc 1 ký tự lệnh
    char readCommand() {
        return bt.read();
    }

    // Các hàm gửi dữ liệu (Dùng template để tự động nhận dạng kiểu số/chữ)
    template <typename T>
    void print(T msg) {
        bt.print(msg);
    }

    template <typename T>
    void print(T msg, int format) {
        bt.print(msg, format); // Hỗ trợ in số thập phân (ví dụ: in 3 số sau dấu phẩy)
    }

    template <typename T>
    void println(T msg) {
        bt.println(msg);
    }

    void println() {
        bt.println();
    }
};