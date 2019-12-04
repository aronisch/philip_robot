#include "Motor.h"

Motor::Motor(uint8_t pwmPinNumber, uint8_t directionPin, bool forw):_pwmPin(pwmPinNumber), _dirPin(directionPin), _forward(forw){
    pinMode(_pwmPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    analogWriteFrequency(_pwmPin, PWM_FREQUENCY);
}

//Speed [-255; 255]
void Motor::setPWMSpeed(double speed) {
    if(speed < 0.0){
        (_forward) ? digitalWrite(_dirPin, LOW) : digitalWrite(_dirPin, HIGH);
    } else {
        (_forward) ? digitalWrite(_dirPin, HIGH) : digitalWrite(_dirPin, LOW);
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

    integralR = integralR > MAX_PWM/_KiR ? MAX_PWM/_KiR : integralR;
    integralR = integralR < MIN_PWM/_KiR ? MIN_PWM/_KiR : integralR;

    integralL = integralL > MAX_PWM/_KiL ? MAX_PWM/_KiL : integralL;
    integralL = integralL < MIN_PWM/_KiL ? MIN_PWM/_KiL : integralL;

    double pTermR = errorR * _KpR;
    double pTermL = errorL * _KpL;

    double iTermR = integralR * _KiR;
    double iTermL = integralL * _KiL;

    double dTermR = (errorR - previousErrorR) / _controlLoopTimeInterval/1000000 * _KdR;
    double dTermL = (errorL - previousErrorL) / _controlLoopTimeInterval/1000000 * _KdL;

    previousErrorR = errorR;
    previousErrorL = errorL;

    double pwmSetPointR = pTermR + iTermR + dTermR;
    double pwmSetPointL = pTermL + iTermL + dTermL;

    pwmSetPointR = pwmSetPointR > MAX_PWM ? MAX_PWM : pwmSetPointR;
    pwmSetPointR = pwmSetPointR < MIN_PWM ? MIN_PWM : pwmSetPointR;

    pwmSetPointL = pwmSetPointL > MAX_PWM ? MAX_PWM : pwmSetPointL;
    pwmSetPointL = pwmSetPointL < MIN_PWM ? MIN_PWM : pwmSetPointL;

    _motorRight->setPWMSpeed(pwmSetPointR);
    _motorLeft->setPWMSpeed(pwmSetPointL);
}