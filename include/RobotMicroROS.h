#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <string.h>
#include <micro_ros_arduino.h>
#include "RobotConfig.h"
#include "RobotLidar.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <kaiaai_msgs/msg/kaiaai_telemetry2.h>
#include <kaiaai_msgs/msg/joint_pos_vel.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ Serial.printf("[Micro-ROS] Error %d at line %d\n", temp_rc, __LINE__); return; }}

class RobotMicroROS {
private:
    const char* ssid = "804 cot cheo";
    const char* password = "VietNam01@";
    const char* agent_ip = "192.168.1.12";
    const size_t agent_port = 8888;          

    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    
    rcl_publisher_t telem_publisher;
    rcl_publisher_t imu_publisher;
    #if ENABLE_DIRECT_ODOM_PUB
    rcl_publisher_t odom_publisher;
    nav_msgs__msg__Odometry odom_msg;
    #endif
    rcl_subscription_t cmd_vel_subscriber;

    kaiaai_msgs__msg__KaiaaiTelemetry2 telem_msg;
    kaiaai_msgs__msg__JointPosVel joint_data[2];
    uint8_t lds_payload_buf[kaiaai_msgs__msg__KaiaaiTelemetry2__lds__MAX_SIZE];

    sensor_msgs__msg__Imu imu_msg;
    geometry_msgs__msg__Twist cmd_vel_msg;

    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float prev_yaw_rad = 0.0f;
    uint32_t telem_seq = 0;
    bool first_odom_step = true;
    bool ros_ready = false; 

    // Bộ định thời kiểm soát tốc độ gửi dữ liệu qua mạng UDP
    unsigned long last_pub_time_ms = 0;
    unsigned long last_rssi_time_ms = 0;
    int8_t cached_rssi = 0;

    static volatile float target_vx;
    static volatile float target_wz;
    static volatile unsigned long last_cmd_time_ms;

    static void cmdVelCallback(const void * msgin) {
        const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
        target_vx = (float)msg->linear.x;
        target_wz = (float)msg->angular.z;
        last_cmd_time_ms = millis();
    }

    void setQuaternion(float yaw_rad, double* qz, double* qw) {
        *qz = sinf(yaw_rad * 0.5f);
        *qw = cosf(yaw_rad * 0.5f);
    }

    void initStaticBuffersAndCovariances() {
        telem_msg.seq = 0;
        telem_msg.wifi_rssi_dbm = 0;
        telem_msg.battery_mv = 12000;

        telem_msg.joint.data = joint_data;
        telem_msg.joint.size = 2;
        telem_msg.joint.capacity = 2;

        telem_msg.lds.data = lds_payload_buf;
        telem_msg.lds.size = 0;
        telem_msg.lds.capacity = kaiaai_msgs__msg__KaiaaiTelemetry2__lds__MAX_SIZE;

        imu_msg.header.frame_id.data = (char*)"imu_link";
        imu_msg.header.frame_id.size = 8;
        imu_msg.header.frame_id.capacity = 9;

        for(int i = 0; i < 9; i++) {
            imu_msg.orientation_covariance[i] = 0.0;
            imu_msg.angular_velocity_covariance[i] = 0.0;
            imu_msg.linear_acceleration_covariance[i] = -1.0;
        }
        imu_msg.orientation_covariance[8] = 0.03;
        imu_msg.angular_velocity_covariance[8] = 0.01;

        #if ENABLE_DIRECT_ODOM_PUB
        odom_msg.header.frame_id.data = (char*)"odom";
        odom_msg.header.frame_id.size = 4;
        odom_msg.header.frame_id.capacity = 5;
        
        odom_msg.child_frame_id.data = (char*)"base_link";
        odom_msg.child_frame_id.size = 9;
        odom_msg.child_frame_id.capacity = 10;

        for(int i = 0; i < 36; i++) {
            odom_msg.pose.covariance[i] = 0.0;
            odom_msg.twist.covariance[i] = 0.0;
        }
        odom_msg.pose.covariance[0] = 0.001;
        odom_msg.pose.covariance[7] = 0.001;
        odom_msg.pose.covariance[14] = 1e6;
        odom_msg.pose.covariance[21] = 1e6;
        odom_msg.pose.covariance[28] = 1e6;
        odom_msg.pose.covariance[35] = 0.03;
        odom_msg.twist.covariance[0] = 0.001;
        odom_msg.twist.covariance[35] = 0.03;
        #endif
    }

public:
    void begin() {
        memset(&telem_msg, 0, sizeof(telem_msg));
        memset(&joint_data, 0, sizeof(joint_data));
        memset(&imu_msg, 0, sizeof(imu_msg));
        memset(&cmd_vel_msg, 0, sizeof(cmd_vel_msg));
        #if ENABLE_DIRECT_ODOM_PUB
        memset(&odom_msg, 0, sizeof(odom_msg));
        #endif

        Serial.println("\n[Micro-ROS] Ket noi Wi-Fi...");
        WiFi.mode(WIFI_STA);
        WiFi.begin(ssid, password);
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }

        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\n[Micro-ROS] ERROR: Wi-Fi Failed!");
            ros_ready = false;
            return;
        }

        set_microros_wifi_transports((char*)ssid, (char*)password, (char*)agent_ip, agent_port);
        delay(500); 

        allocator = rcl_get_default_allocator();

        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
        RCCHECK(rclc_node_init_default(&node, "pet", "", &support));

        RCCHECK(rclc_publisher_init_best_effort(
            &telem_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(kaiaai_msgs, msg, KaiaaiTelemetry2),
            "/telemetry"));

        RCCHECK(rclc_publisher_init_best_effort(
            &imu_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "/imu/data"));

        #if ENABLE_DIRECT_ODOM_PUB
        RCCHECK(rclc_publisher_init_best_effort(
            &odom_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/odom"));
        #endif

        RCCHECK(rclc_subscription_init_best_effort(
            &cmd_vel_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel"));

        RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
        RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &RobotMicroROS::cmdVelCallback, ON_NEW_DATA));

        initStaticBuffersAndCovariances();
        rmw_uros_sync_session(1000);
        
        Serial.println("[Micro-ROS] Telemetry2 Ready (Rate-limited to 20Hz)");
        ros_ready = true;
    }

    void spinSome() {
        if (!ros_ready) return;
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    }

    void getCmdVel(float &vx, float &wz) {
        if (millis() - last_cmd_time_ms > 500) {
            vx = 0.0f;
            wz = 0.0f;
        } else {
            vx = target_vx;
            wz = target_wz;
        }
    }

    void publishData(float v_x,
                     float v_theta,
                     float yaw_deg,
                     float gyro_z_deg,
                     float left_pos_rad,
                     float right_pos_rad,
                     float left_vel_rad,
                     float right_vel_rad,
                     float dt) {
        if (!ros_ready) return;

        float yaw_rad = yaw_deg * (PI_VAL / 180.0f);
        float gyro_z_rad = gyro_z_deg * (PI_VAL / 180.0f);

        if (first_odom_step) {
            prev_yaw_rad = yaw_rad;
            first_odom_step = false;
        }

        // Tích lũy vị trí Odometry liên tục ở 50Hz để không mất độ chính xác
        float delta_s = v_x * dt;
        float mid_yaw = (prev_yaw_rad + yaw_rad) * 0.5f;
        prev_yaw_rad = yaw_rad;

        pose_x += delta_s * cosf(mid_yaw);
        pose_y += delta_s * sinf(mid_yaw);

        // GIỚI HẠN TẦN SỐ GỬI UDP: Chỉ gửi qua Wi-Fi mỗi 20ms (20Hz)
        unsigned long current_time_ms = millis();
        if (current_time_ms - last_pub_time_ms < 20) {
            return;
        }
        last_pub_time_ms = current_time_ms;

        // Cập nhật RSSI mỗi 1 giây/lần
        if (current_time_ms - last_rssi_time_ms > 1000) {
            cached_rssi = (int8_t)WiFi.RSSI();
            last_rssi_time_ms = current_time_ms;
        }

        int64_t time_ms = rmw_uros_epoch_millis();
        int32_t sec = time_ms / 1000;
        uint32_t nanosec = (time_ms % 1000) * 1000000;

        // 1. ĐÓNG GÓI & GỬI KAIAAI TELEMETRY2
        telem_msg.stamp.sec = sec;
        telem_msg.stamp.nanosec = nanosec;
        telem_msg.seq = telem_seq++;

        telem_msg.odom_pos_x = pose_x;
        telem_msg.odom_pos_y = pose_y;
        telem_msg.odom_pos_yaw = yaw_rad;
        telem_msg.odom_vel_x = v_x;
        telem_msg.odom_vel_yaw = gyro_z_rad;

        joint_data[0].pos = left_pos_rad;
        joint_data[0].vel = left_vel_rad;
        joint_data[1].pos = right_pos_rad;
        joint_data[1].vel = right_vel_rad;

        telem_msg.wifi_rssi_dbm = cached_rssi;

        bool scan_start_hint = false;
        size_t lds_len = RobotLidar::popLdsData(
            lds_payload_buf,
            250, 
            scan_start_hint);
        telem_msg.lds.size = lds_len;
        telem_msg.scan_start_hint = scan_start_hint;

        rcl_publish(&telem_publisher, &telem_msg, NULL);

        // 2. ĐÓNG GÓI & GỬI IMU
        double qz = 0.0, qw = 1.0;
        setQuaternion(yaw_rad, &qz, &qw);

        imu_msg.header.stamp.sec = sec;
        imu_msg.header.stamp.nanosec = nanosec;
        imu_msg.angular_velocity.x = 0.0;
        imu_msg.angular_velocity.y = 0.0;
        imu_msg.angular_velocity.z = gyro_z_rad;
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = qz;
        imu_msg.orientation.w = qw;

        rcl_publish(&imu_publisher, &imu_msg, NULL);

        #if ENABLE_DIRECT_ODOM_PUB
        odom_msg.header.stamp.sec = sec;
        odom_msg.header.stamp.nanosec = nanosec;
        odom_msg.pose.pose.position.x = pose_x;
        odom_msg.pose.pose.position.y = pose_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = imu_msg.orientation;
        odom_msg.twist.twist.linear.x = v_x;
        odom_msg.twist.twist.angular.z = v_theta;
        rcl_publish(&odom_publisher, &odom_msg, NULL);
        #endif
    }
};

volatile float RobotMicroROS::target_vx = 0.0f;
volatile float RobotMicroROS::target_wz = 0.0f;
volatile unsigned long RobotMicroROS::last_cmd_time_ms = 0;