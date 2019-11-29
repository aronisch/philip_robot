#include "Motor.h"

Motor::Motor(uint8_t pwmPinNumber, uint8_t directionPin):_pwmPin(pwmPinNumber), _dirPin(directionPin){
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    analogWriteFrequency(_pwmPin, PWM_FREQUENCY);
}

//Speed [-255; 255]
void Motor::setPWMSpeed(double speed) {
    if(speed < 0.0){
        digitalWrite(_dirPin, LOW);
    } else {
        digitalWrite(_dirPin, HIGH);
    }
    speed = speed > MAX_PWM ? MAX_PWM : speed;
    speed = speed < MIN_PWM ? MIN_PWM : speed;
    analogWrite(_pwmPin, (uint8_t)abs(speed));
}

void MotorPID::speedControlLoop(double currentSpeedR, double currentSpeedL){
    static double integralR = 0;
    static double integralL = 0;
    static double previousErrorR = 0;
    static double previousErrorL = 0;

    double errorR = _setpointSpeedR - currentSpeedR;
    double errorL = _setpointSpeedL - currentSpeedL;

    integralR += errorR * _controlLoopTimeInterval/1000000;
    integralL += errorL * _controlLoopTimeInterval/1000000;

    integralR = integralR > MAX_PWM/_Ki ? MAX_PWM/_Ki : integralR;
    integralR = integralR < MIN_PWM/_Ki ? MIN_PWM/_Ki : integralR;

    integralL = integralL > MAX_PWM/_Ki ? MAX_PWM/_Ki : integralL;
    integralL = integralL < MIN_PWM/_Ki ? MIN_PWM/_Ki : integralL;

    double pTermR = errorR * _Kp;
    double pTermL = errorL * _Kp;

    double iTermR = integralR * _Ki;
    double iTermL = integralL * _Ki;

    double dTermR = (errorR - previousErrorR) / _controlLoopTimeInterval/1000000 * _Kd;
    double dTermL = (errorL - previousErrorL) / _controlLoopTimeInterval/1000000 * _Kd;

    double pwmSetPointR = pTermR + iTermR + dTermR;
    double pwmSetPointL = pTermL + iTermL + dTermL;

    pwmSetPointR = pwmSetPointR > MAX_PWM ? MAX_PWM : pwmSetPointR;
    pwmSetPointR = pwmSetPointR < MIN_PWM ? MIN_PWM : pwmSetPointR;

    pwmSetPointL = pwmSetPointL > MAX_PWM ? MAX_PWM : pwmSetPointL;
    pwmSetPointL = pwmSetPointL < MIN_PWM ? MIN_PWM : pwmSetPointL;

    _motorRight->setPWMSpeed(pwmSetPointR);
    _motorLeft->setPWMSpeed(pwmSetPointL);
}