#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "QuadDecoder.h"

class Odometry {
private:
    QuadDecoder _decoderRight = QuadDecoder(2);
    QuadDecoder _decoderLeft = QuadDecoder(1);

    uint32_t _ticksPerRev;
    double _wheelRadius; //mm
    double _axleDistance; //mm
    double _updateTimeIntervalUS;

    double _speedRight = 0;
    double _speedLeft = 0;

    double _positionX = 0;
    double _positionY = 0;
    double _positionTheta = 0;
public:
    Odometry(uint32_t ticksRev, double wheelR, double axleD, double timeIntervalUS);
    void updateSpeedsAndPositions();
    void resetPosition();
    double getSpeedRight(){ return _speedRight; };
    double getSpeedLeft(){ return _speedLeft; };
    double getPositionX(){ return _positionX; };
    double getPositionY(){ return _positionY; };
    double getPositionTheta(){ return _positionTheta; };
};

#endif