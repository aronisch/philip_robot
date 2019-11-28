#pragma once
#include <Wire.h>
//Pin Structure of Philip's Teensy

//Motor Encoder
#define ENCODER_A1 (1)
#define ENCODER_B1 (2)
#define ENCODER_A2 (3)
#define ENCODER_B2 (5)

//Motor Control
#define MOTOR_PWM_1 (15)
#define MOTOR_PWM_2 (14)
#define MOTOR_DIR_1 (13)
#define MOTOR_DIR_2 (20)
#define MOTOR_ENABLE (21)

//Motor Current Sensing
#define MOTOR_FB_1 (A9)
#define MOTOR_FB_2 (A8)

//Ultrasonic Sensors
#define US_TRIG_LEFT (0)
#define US_TRIG_RIGHT (10)
#define US_ECHO_LEFT (11)
#define US_ECHO_RIGHT (12)

//Servos
#define SERVO_PLATE (4) //Bloquant 80 - Ouvert 170
#define SERVO_GRIP_ROTATION (6) //Prise 170 - Depose 10
#define SERVO_GRIP (9)  //Ouvert 25 - Ferme 110

//Lidar Sensors
//Wire --> Left Lidar | Wire1 --> Right Lidar