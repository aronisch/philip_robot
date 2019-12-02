#ifndef __LIDAR_H__
#define __LIDAR_H__

#include "Adafruit_VL53L0X.h"

class Lidar : public Adafruit_VL53L0X
{
    private:
        uint16_t dist = 0;
        VL53L0X_RangingMeasurementData_t measure;
    public:
        uint16_t getDist();
};

#endif