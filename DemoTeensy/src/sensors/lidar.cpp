 #include "lidar.h"

 uint16_t Lidar::getDist(){
    this->rangingTest(this->measure, false);
     if (this->measure.RangeStatus != 4) {  // phase failures have incorrect data
        return this->measure.RangeMilliMeter
    else 
        return 0xFFFF;
 }