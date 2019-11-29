#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Odometry.h"

#define MAX_PWM (255.0f)
#define MIN_PWM (-255.0f)

// Motor control class

class Motor {
private:
    uint8_t _pwmPin;
    uint8_t _dirPin;
public:
    Motor(uint8_t pwmPinNumber, uint8_t directionPin);
    void setPWMSpeed(double speed);
};

class MotorPID {
private:
    Motor &_motorLeft;
    Motor &_motorRight;
    Odometry &_odo;
    double _controlLoopTimeInterval;
    float _Kp;
    float _Ki;
    float _Kd;
public:
    //Need to implement a wrapper function in the main which runs the speedControlLoop of motorPID
    MotorPID(Motor &mLeft, Motor &mRight, Odometry &odo, double timeInterval, float Kp, float Ki, float Kd): _motorLeft(mLeft), _motorRight(mRight), _odo(odo), _controlLoopTimeInterval(timeInterval), _Kp(Kp), _Ki(Ki), _Kd(Kd){};
    void speedControlLoop(double currentSpeedR, double currentSpeedL, double setpointSpeedR, double setpointSpeedL);
};

#endif