// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MYROBOT_CONFIG_H
#define MYROBOT_CONFIG_H

#define LED_PIN 2
#define ESP32
//#define SOC_PCNT_SUPPORTED
#define BAUDRATE 115200
#define LINO_BASE DIFFERENTIAL_DRIVE 

#define USE_GENERIC_2_IN_MOTOR_DRIVER 

#define USE_MPU6050_IMU
#define SDA_PIN 21 // specify I2C pins for esp32
#define SCL_PIN 22

// LEFT MOTOR
#define K_P_L 0.6                             
#define K_I_L 0.8                             
#define K_D_L 0.5

// RIGHT MOTOR
#define K_P_R 0.5                             
#define K_I_R 0.8                             
#define K_D_R 0.5

#define MOTOR_MAX_RPM 143       
#define MAX_RPM_RATIO_L 0.8
#define MAX_RPM_RATIO_R 0.7

#define MOTOR_OPERATING_VOLTAGE 12
#define MOTOR_POWER_MAX_VOLTAGE 12
#define MOTOR_POWER_MEASURED_VOLTAGE 12.0            
#define COUNTS_PER_REV1 628 //960
#define COUNTS_PER_REV2 663 //960
#define COUNTS_PER_REV3 240
#define COUNTS_PER_REV4 240
#define WHEEL_DIAMETER 0.136               
#define LR_WHEELS_DISTANCE 0.305            
#define PWM_BITS 10                         
#define PWM_FREQUENCY 20000

// Fixed pin numbers for ESP32-WROOM-32D 38 PIN VERSION


//LEFT Motor Pins
#define MOTOR1_PWM 14
#define MOTOR1_IN_A 12
#define MOTOR1_IN_B 13 
#define MOTOR1_INV false

//RIGHT Motor Pins
#define MOTOR2_PWM 25
#define MOTOR2_IN_A 27
#define MOTOR2_IN_B 26 
#define MOTOR2_INV false

///LEFT ENCODER PINS
#define MOTOR1_ENCODER_A 19
#define MOTOR1_ENCODER_B 18 
#define MOTOR1_ENCODER_INV false 

// RIGHT ENCODER PINS 
#define MOTOR2_ENCODER_A 15
#define MOTOR2_ENCODER_B 23 
#define MOTOR2_ENCODER_INV false 

#define PWM_MAX pow(2, PWM_BITS) - 1
#define PWM_MIN -(pow(2, PWM_BITS) - 1)

#define AGENT_IP { 192, 168, 1, 117 }  // eg IP of the desktop computer
#define AGENT_PORT 8888
// Enable WiFi with null terminated list of multiple APs SSID and password
#define WIFI_AP_LIST {{"wifi_udev_robot", "0801629807dev"}, {NULL}}
#define WIFI_MONITOR 2 // min. period to send wifi signal strength to syslog
// #define USE_ARDUINO_OTA
//#define USE_SYSLOG
#define SYSLOG_SERVER { 192, 168, 1, 117 }  // eg IP of the desktop computer
#define SYSLOG_PORT 514
#define DEVICE_HOSTNAME "linorobot2"
#define APP_NAME "hardware"
#endif
