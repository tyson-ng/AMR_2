#define APP_MODE 3 // 1: Test Lidar | 2: Test Bluetooth | 3: Full System micro-ROS

#include <Arduino.h>
#include "RobotConfig.h"
#include "RobotEncoder.h"
#include "RobotImu.h"
#include "RobotMotors.h"
#include "RobotPID.h"
#include "RobotMicroROS.h" 
#include "RobotTester.h"
#include "RobotLidar.h"

#if APP_MODE == 2
#include "RobotBluetooth.h"  
RobotBluetooth bt;
#endif

RobotEncoder leftMotor;
RobotEncoder rightMotor;
RobotImu imu;
RobotMotors motors;
RobotTester tester;
RobotLidar robotLidar;
RobotMicroROS ros_node; 

// PID vòng trong (Velocity) - Có thể mở rộng giới hạn lên [-255, 255] khi chạy thực tế
RobotPID pidLeft(470.15, 8286.83, 0.0, -255.0, 255.0);
RobotPID pidRight(470.15, 8286.83, 0.0, -255.0, 255.0);

// PID vòng ngoài (Heading Cascade)
HeadingPID pidHeading(0.015, 0.0005, 0.005, 0.15);

void TaskLidar(void *pvParameters) {
  HardwareSerial& LidarSerial = RobotLidar::getSerial();

  for (;;) {
    #if APP_MODE == 3
      // Hút toàn bộ byte thô có trong buffer UART2 đưa thẳng vào RingBuffer
      while (LidarSerial.available()) {
        uint8_t b = (uint8_t)LidarSerial.read();
        RobotLidar::pushByte(b);
      }
      vTaskDelay(pdMS_TO_TICKS(5)); // Nghỉ 5ms nhường CPU Core 0 cho Wi-Fi
    #elif APP_MODE == 1
      robotLidar.update();
      vTaskDelay(pdMS_TO_TICKS(1));
    #else
      vTaskDelay(pdMS_TO_TICKS(1000));
    #endif
  }
}

void TaskLogicControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // Chu kỳ chuẩn 50Hz (dt = 0.02s)

  for (;;) {
    #if APP_MODE == 2
      tester.updateBluetooth(bt, leftMotor, rightMotor, imu, motors, pidLeft, pidRight, pidHeading);
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #elif APP_MODE == 3
      tester.updateROS(leftMotor, rightMotor, imu, motors, pidLeft, pidRight, pidHeading, ros_node);
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    #else
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      xLastWakeTime = xTaskGetTickCount();
    #endif
  }
}

void setup() {
  Serial.begin(115200); 
  Serial.setTimeout(5);
  
  #if APP_MODE == 2 || APP_MODE == 3
    leftMotor.begin(ENCA_A, ENCA_B, false);
    rightMotor.begin(ENCB_A, ENCB_B, true);
    imu.begin();
    motors.begin();
  #endif

  #if APP_MODE == 2
    bt.begin("ESP32_Robot_Test");
    Serial.println(">> MODE 2: BLUETOOTH TEST MODE <<");
  #endif

  #if APP_MODE == 1 || APP_MODE == 3
    robotLidar.begin(); 
  #endif

  #if APP_MODE == 3
    ros_node.begin(); 
    Serial.println(">> MODE 3: FULL SYSTEM MICRO-ROS (BT DISABLED) <<");
  #endif

  xTaskCreatePinnedToCore(TaskLidar, "LidarTask", 8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(TaskLogicControl, "LogicTask", 8192, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL); 
}