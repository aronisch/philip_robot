#ifndef __ULTRASONICSENSOR_H__
#define __ULTRASONICSENSOR_H__

class UltrasonicSensor
{
  private : 
    int _echoPin;
    int _trigPin;

  public :
    UltrasonicSensor(int echoPin, int trigPin);
    int getDist();

}


#endif