#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "QuadDecoder.h"

class Odometry {
private:
    QuadDecoder _decoderRight = QuadDecoder(1, 0);
    QuadDecoder _decoderLeft = QuadDecoder(2, 0);

    uint32_t _ticksPerRev;
    double _wheelRadius; //mm
    double _axleDistance; //mm
    double _updateTimeIntervalUS;

    double _speedRight;
    double _speedLeft;

    double _positionX = 0;
    double _positionY = 0;
    double _positionTheta = 0;
public:
    Odometry(uint32_t ticksRev, double wheelR, double axleD, double timeIntervalUS):_ticksPerRev(ticksRev), _wheelRadius(wheelR), _axleDistance(axleD), _updateTimeIntervalUS(timeIntervalUS){};
    void updateSpeedsAndPositions();
    void resetPosition();
    double getSpeedRight(){ return _speedRight; };
    double getSpeedLeft(){ return _speedLeft; };
    double getPositionX(){ return _positionX; };
    double getPositionY(){ return _positionY; };
};

#endif