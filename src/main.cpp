#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotEncoder.h"
#include "RobotImu.h"
#include "RobotMotors.h"
#include "RobotPID.h"
#include "RobotBluetooth.h"
#include "RobotTester.h"

RobotEncoder leftMotor;
RobotEncoder rightMotor;
RobotImu imu;
RobotMotors motors;
RobotBluetooth bt;
RobotTester tester;

// Điền Kp, Ki thu được từ MATLAB vào đây
RobotPID pidLeft(470.15, 8286.83, 0.0, -150.0, 150.0);
RobotPID pidRight(470.15, 8286.83, 0.0, -150.0, 150.0);

// Khởi tạo vòng ngoài (Kp = 0.015, Ki = 0.0005, Kd = 0.005, Max Delta V = 0.15 m/s)
HeadingPID pidHeading(0.015, 0.0005, 0.005, 0.15);



// ---------------------------------------------------------
void TaskCommunication(void *pvParameters) {
  for (;;) {
    // Chỉ 1 dòng duy nhất lo tất cả việc: Đọc lệnh, Test Động cơ, Tính PID, Gửi Bluetooth
    tester.update(bt, leftMotor, rightMotor, imu, motors, pidLeft, pidRight, pidHeading);
    
    // Luồng này phải chạy thật nhanh (5ms/lần) để kịp phục vụ bộ đếm giờ trong Tester
    vTaskDelay(5 / portTICK_PERIOD_MS); 
  }
}

// ---------------------------------------------------------
void setup() {
  Serial.begin(115200); 
  
// Đảo ngược lại trạng thái của cả 2 bánh
  leftMotor.begin(ENCA_A, ENCA_B, false);
  rightMotor.begin(ENCB_A, ENCB_B, true);

  imu.begin();
  motors.begin();
  bt.begin("ESP32_Robot_Test");

  Serial.println(">> ROBOT TESTER DA SAN SANG <<");
  Serial.println("- Go 'P150' de test PWM tho cho MATLAB");
  Serial.println("- Go 'V0.15' de test PID tu dong");
  Serial.println("- Mo App Bluetooth bam 1,2,3 de xem cam bien");

  xTaskCreatePinnedToCore(TaskCommunication, "LogicTask", 8192, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}