#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define MAX_PWM (255.0f)
#define MIN_PWM (-255.0f)
#define PWM_FREQUENCY (50000)

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
    Motor *_motorLeft;
    Motor *_motorRight;
    double _controlLoopTimeInterval;
    float _Kp;
    float _Ki;
    float _Kd;

    double _setpointSpeedR = 0;
    double _setpointSpeedL = 0;
public:
    //Need to implement a wrapper function in the main which runs the speedControlLoop of motorPID
    MotorPID(Motor *mLeft, Motor *mRight, double timeInterval, float Kp, float Ki, float Kd): _motorLeft(mLeft), _motorRight(mRight), _controlLoopTimeInterval(timeInterval), _Kp(Kp), _Ki(Ki), _Kd(Kd){};
    void speedControlLoop(double currentSpeedR, double currentSpeedL);
    void setSpeedRight(double speed){ _setpointSpeedR = speed; };
    void setSpeedLeft(double speed){ _setpointSpeedL = speed; };
};

#endif