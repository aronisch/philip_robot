 #include "lidar.h"

 uint16_t Lidar::getDist(){
    this->rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        return measure.RangeMilliMeter;
    } else 
        return 0xFFFF;
 }