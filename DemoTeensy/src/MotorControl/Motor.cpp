#include "Motor.h"

Motor::Motor(uint8_t pwmPinNumber, uint8_t directionPin):_pwmPin(pwmPinNumber), _dirPin(directionPin){
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
}

//Speed [-255; 255]
void Motor::setPWMSpeed(double speed) {
    if(speed > 0.0){
        digitalWrite(_dirPin, LOW);
    } else {
        digitalWrite(_dirPin, HIGH);
    }
    speed = speed > MAX_PWM ? MAX_PWM : speed;
    speed = speed < MIN_PWM ? MIN_PWM : speed;
    analogWrite(_pwmPin, (uint8_t)abs(speed));
}

void MotorPID::speedControlLoop(double currentSpeedR, double currentSpeedL, double setpointSpeedR, double setpointSpeedL){
    static double integralR = 0;
    static double integralL = 0;
    static double previousR = 0;
    static double previousL = 0;

    double errorR = setpointSpeedR - currentSpeedR;
    double errorL = setpointSpeedL - currentSpeedL;

    integralR += errorR * _controlLoopTimeInterval;
    integralL += errorL * _controlLoopTimeInterval;

    double pTermR = errorR * _Kp;
    double pTermL = errorL * _Kp;

    double iTermR = integralR * _Ki;
    double iTermL = integralL * _Ki;

    double dTermR = (currentSpeedR - previousR) / _controlLoopTimeInterval * _Kd;
    double dTermL = (currentSpeedL - previousL) / _controlLoopTimeInterval * _Kd;

    double pwmSetPointR = pTermR + iTermR + dTermR;
    double pwmSetPointL = pTermL + iTermL + dTermL;

    pwmSetPointR = pwmSetPointR > MAX_PWM ? MAX_PWM : pwmSetPointR;
    pwmSetPointR = pwmSetPointR < MIN_PWM ? MIN_PWM : pwmSetPointR;

    pwmSetPointL = pwmSetPointL > MAX_PWM ? MAX_PWM : pwmSetPointL;
    pwmSetPointL = pwmSetPointL < MIN_PWM ? MIN_PWM : pwmSetPointL;

    _motorRight.setPWMSpeed(pwmSetPointR);
    _motorLeft.setPWMSpeed(pwmSetPointL);
}