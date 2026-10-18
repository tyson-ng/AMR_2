#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotEncoder.h"
#include "RobotImu.h"
#include "RobotBluetooth.h" 
#include "RobotTester.h"
#include "RobotLidar.h" 

// ==========================================
// CÔNG TẮC CHUYỂN CHẾ ĐỘ (Sửa số ở đây)
// 0: Test mâm gầm (Encoder + IMU + Bluetooth)
// 1: Test YDLidar X3 (In lên Serial Monitor)
#define APP_MODE 1 
// ==========================================

// --- KHỞI TẠO MODULE PHẦN CỨNG ---
RobotEncoder leftMotor;
RobotEncoder rightMotor;
RobotImu imu;
RobotBluetooth bt;
RobotLidar robotLidar; 
RobotTester tester;

// ---------------------------------------------------------
void TaskReadSensors(void *pvParameters) {
  for (;;) {
    #if APP_MODE == 0
      imu.update(); 
      vTaskDelay(10 / portTICK_PERIOD_MS); 
      
    #elif APP_MODE == 1
      robotLidar.update(); 
      vTaskDelay(1 / portTICK_PERIOD_MS); 
    #endif
  }
}

// ---------------------------------------------------------
void TaskCommunication(void *pvParameters) {
  for (;;) {
    #if APP_MODE == 0
      tester.update(bt, leftMotor, rightMotor, imu);
      vTaskDelay(200 / portTICK_PERIOD_MS); 

    #elif APP_MODE == 1
      // Lidar test không dùng Bluetooth, cho luồng này ngủ
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
    #endif
  }
}

// ---------------------------------------------------------
void setup() {
  Serial.begin(115200);

  #if APP_MODE == 0
    Serial.println(">> KHOI DONG: TEST ENCODER + IMU <<");
    leftMotor.begin(ENCA_A, ENCA_B);
    rightMotor.begin(ENCB_A, ENCB_B);
    imu.begin();
    bt.begin("ESP32_Robot_Test");

  #elif APP_MODE == 1
    Serial.println(">> KHOI DONG: TEST YDLIDAR X3 <<");
    robotLidar.begin();
  #endif

  xTaskCreatePinnedToCore(TaskReadSensors, "SensorTask", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskCommunication, "LogicTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}