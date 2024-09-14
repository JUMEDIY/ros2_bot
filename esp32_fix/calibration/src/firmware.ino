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

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include "config.h"
#include "motor.h"
#include "pid.h"

#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS

#include "encoder.h"
#include "kinematics.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif
#define SAMPLE_TIME 20 // seconds

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P_L, K_I_L, K_D_L);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P_R, K_I_R, K_D_R);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO_L,
    MAX_RPM_RATIO_R,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE);

long long int counts_per_rev[2];
int total_motors = 2;
Motor *motors[2] = {&motor1_controller, &motor2_controller };
Encoder *encoders[2] = {&motor1_encoder, &motor2_encoder };
PID *pids[2] = {&motor1_pid, &motor2_pid };
String labels[2] = {"FRONT LEFT - M1: ", "FRONT RIGHT - M2: "};

void setup()
{
#ifdef BOARD_INIT
    BOARD_INIT;
#endif

    Serial.begin(BAUDRATE);
    while (!Serial)
    {
    }
    Serial.println("Sampling process will spin the motors at its maximum RPM.");
    Serial.println("Please ensure that the robot is ELEVATED and there are NO OBSTRUCTIONS to the wheels.");
    Serial.println("");
    Serial.println("Type 'spin' and press enter to spin the motors.");
    Serial.println("Type 'sample' and press enter to spin the motors with motor summary.");
    Serial.println("Type 'ticks' and press enter to measure ticks per revolution of the motors.");
    Serial.println("Type 'test' and press enter to spin the using cmd_vel and observe PID in action.");
    Serial.println("Press enter to clear command.");
    Serial.println("");
}

void loop()
{
    static String cmd = "";

    while (Serial.available())
    {
        char character = Serial.read();
        cmd.concat(character);
        Serial.print(character);
        delay(1);
        if (character == '\r' and cmd.equals("spin\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            sampleMotors(0);
        }
        else if (character == '\r' and cmd.equals("sample\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            sampleMotors(1);
        }
        else if (character == '\r' and cmd.equals("ticks\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            testMotorForTicksPerRevolution();
        }
        else if (character == '\r' and cmd.equals("test\r"))
        {
            cmd = "";
            Serial.println("\r\n");
            testMotorsWithCmdVel();
        }
        else if (character == '\r')
        {
            Serial.println("");
            cmd = "";
        }
    }
}
void testMotorForTicksPerRevolution()
{
    if (Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }
    int PWM_FOR_TEST = 200;
    unsigned long start_time = micros();
    unsigned long last_status = micros();
    bool show_incremental_tick_count = false;
    

    Serial.println("Please ensure that the robot is ELEVATED and there are NO OBSTRUCTIONS to the wheels.");
    Serial.println("ticks test will run each motor at slow speed one motor at a time.");
    Serial.println("count the number of revolutions visually and make a note of final tick count for each motor along with number of revolutions observed.");
    Serial.println("Number of ticks for each motor can be calculated with following formula:");
    Serial.println("ticks per rev = final tick count/number of revolutions counted");
    Serial.println("Press enter to continue to tick count test.");
    Serial.println("");
    char character = Serial.read();

    for (int i = 0; i < total_motors; i++)
    {
        Serial.print(labels[i]);
        while (true)
        {
            if (micros() - start_time >= SAMPLE_TIME * 1000000)
                {
                    //Serial.println("STOP motor");
                    motors[i]->brake();
                    start_time = micros();

                    break;
                }


            // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
            // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
            float current_tick_count = encoders[i]->read();
            // set show_incremental_tick_count = true at beginning of function if you want to print incremental values
            // otherwise it will only print final total tick count for each motor
            if(show_incremental_tick_count){
                Serial.print("current_tick_count:: ");
                Serial.println(current_tick_count);
            }else{
                Serial.print(".");

            }
            // run the motor with low pwm so it rotates slowly and revolutions can be counted visually
            // If the motor is rotating too fast change value of PWM_FOR_TEST at beginning of this function
            int pwm = PWM_FOR_TEST; 
            motors[i]->spin(pwm);
            delay(1000);

        }
        Serial.println("");
        Serial.print("final_tick_count for ");
        Serial.print(labels[i]);
        Serial.print(" = ");
        float final_tick_count = encoders[i]->read();
        Serial.println(final_tick_count);
        Serial.println("=============");

    }

}
void testMotorsWithCmdVel()
{
    if (Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }
    geometry_msgs__msg__Twist twist_msg;
    // set test twist message with x=0.5 fwd full speed
    twist_msg.linear.x = 0.5;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;
    unsigned long start_time = micros();
    unsigned long last_status = micros();
    for (int i = 0; i < total_motors; i++)
    {
        while (true)
        {
            if (micros() - start_time >= SAMPLE_TIME * 1000000)
                {
                    Serial.println("STOP motor");
                    motors[i]->brake();
                    Serial.println("=============");
                    start_time = micros();

                    break;
                }

            // do all speed calculations
            // get the required rpm for each motor based on required velocities, and base used
            Kinematics::rpm req_rpm = kinematics.getRPM(
                twist_msg.linear.x,
                twist_msg.linear.y,
                twist_msg.angular.z);
            float req_rpm_val = 0;
            switch (i)
            {
            case 0:
                req_rpm_val = req_rpm.motor1;
                break;
            case 1:
                req_rpm_val = req_rpm.motor2;
                break;
            
            default:
                break;
            }
            float current_rpm = encoders[i]->getRPM();

            // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
            // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
            Serial.print("req_rpm:: ");
            Serial.print(req_rpm_val);
            Serial.print(" current_rpm:: ");
            Serial.print(current_rpm);
            
            int pwm = pids[i]->compute(req_rpm_val, current_rpm);
            Serial.print(" pwm:: ");
            Serial.println(pwm);
            motors[i]->spin(pwm);
            delay(100);

        }
    }
}
void sampleMotors(bool show_summary)
{
    if (Kinematics::LINO_BASE == Kinematics::DIFFERENTIAL_DRIVE)
    {
        total_motors = 2;
    }

    float measured_voltage = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    float scaled_max_rpm = ((measured_voltage / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM);
    float total_rev = scaled_max_rpm * (SAMPLE_TIME / 60.0);
    
    for (int i = 0; i < total_motors; i++)
    {
        Serial.print("SPINNING ");
        Serial.print(labels[i]);

        unsigned long start_time = micros();
        unsigned long last_status = micros();

        encoders[i]->write(0);
        while (true)
        {
            if (micros() - start_time >= SAMPLE_TIME * 1000000)
            {
                // Serial.println("STOP");

                motors[i]->spin(0);
                Serial.println("");
                break;
            }

            if (micros() - last_status >= 1000000)
            {
                last_status = micros();
                Serial.print(".");
            }
            delay(1); // Fix: without this small delay the motors don't spin
            motors[i]->spin(PWM_MAX);
        }
        // Serial.println("before encoder read");
        counts_per_rev[i] = encoders[i]->read() / total_rev;
    }
    if (show_summary)
        printSummary();
}

void printSummary()
{
    Serial.println("\r\n================MOTOR ENCODER READINGS================");
    Serial.print(labels[0]);
    Serial.print(encoders[0]->read());
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(encoders[1]->read());

    
    Serial.println("================COUNTS PER REVOLUTION=================");
    Serial.print(labels[0]);
    Serial.print(counts_per_rev[0]);
    Serial.print(" ");

    Serial.print(labels[1]);
    Serial.println(counts_per_rev[1]);

    
    Serial.println("====================MAX VELOCITIES====================");
    float max_rpm = kinematics.getMaxRPM();

    Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
    Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm, -max_rpm, max_rpm);

    Serial.print("Linear Velocity: +- ");
    Serial.print(max_linear.linear_x);
    Serial.println(" m/s");

    Serial.print("Angular Velocity: +- ");
    Serial.print(max_angular.angular_z);
    Serial.println(" rad/s");
}

