#include "Odometry.h"
#include <math.h>

Odometry::Odometry(uint32_t ticksRev, double wheelR, double axleD, double timeIntervalUS):_ticksPerRev(ticksRev), _wheelRadius(wheelR), _axleDistance(axleD), _updateTimeIntervalUS(timeIntervalUS){
    _decoderRight.begin(0xFFFFFFFF, timeIntervalUS);
    _decoderLeft.begin(0xFFFFFFFF, timeIntervalUS);
    Serial.println("Odometry Initialized");
}

void Odometry::resetPosition(){
    _positionX = 0;
    _positionY = 0;
    _positionTheta = 0;
}

void Odometry::updateSpeedsAndPositions(){
    static uint32_t _previousTicksRight = 0;
    static uint32_t _previousTicksLeft = 0;

    uint32_t currentTicksRight = _decoderRight.getCount();
    uint32_t currentTicksLeft = _decoderLeft.getCount();
    /*Serial.print("Ticks R: ");
    Serial.print(currentTicksRight);
    Serial.print(" - Ticks L: ");
    Serial.println(currentTicksLeft);*/

    int32_t ticksDifferenceRight = (int32_t)currentTicksRight - (int32_t)_previousTicksRight;
    int32_t ticksDifferenceLeft = (int32_t)currentTicksLeft - (int32_t)_previousTicksLeft;
    /*Serial.print("Ticks Diff R:");
    Serial.print(ticksDifferenceRight);
    Serial.print(" - Ticks Diff L");
    Serial.println(ticksDifferenceLeft);*/
    double displacementRight = -(double)(ticksDifferenceRight) * 2 * M_PI * _wheelRadius / _ticksPerRev;
    double displacementLeft = (double)(ticksDifferenceLeft) * 2 * M_PI * _wheelRadius / _ticksPerRev;

    _previousTicksRight = currentTicksRight;
    _previousTicksLeft = currentTicksLeft;

    _speedRight = (displacementRight) * 1000000 / (_updateTimeIntervalUS);
    _speedLeft = (displacementLeft) * 1000000 / (_updateTimeIntervalUS);

    double deltaS = (displacementRight + displacementLeft) / 2;
    double deltaTheta = (displacementRight - displacementLeft)/_axleDistance;
    
    _positionX += deltaS * cosl(_positionTheta + deltaTheta/2);
    _positionY += deltaS * sinl(_positionTheta + deltaTheta/2);
    _positionTheta += deltaTheta;
}