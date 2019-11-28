#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "pin.h"

//Simple class to control servo as actuators
//This class limit the servo range to protect hardware

class ServoAct : public Servo 
{
  private : 
    uint8_t _maxPosition;
    uint8_t _minPosition;
    uint8_t _defaultPosition;

  public :

    //constructor
    ServoAct(uint8_t min = 0, uint8_t max = 180, uint8_t def = 90);

    void write(uint8_t pos);

    void writeMax() {   write(_maxPosition);   };
    void writeMin() {   write(_minPosition);   };

    void writeDefault()  {   write(_defaultPosition);   };


};

//Class used to describe and control the robotic arm at the front of the robot
class SimpleArm
{ 
  private:
    //Arm's constants
    static const uint8_t PLATE = 170;
    static const uint8_t FRONT = 10; 
    static const uint8_t MIDDLE = 90;
    
    //gripper's constants
    static const uint8_t OPEN = 25;
    static const uint8_t CLOSED = 110; 

    ServoAct _grip;
    ServoAct _rotat;
  public: 
    SimpleArm(uint8_t gripperPin, uint8_t armPin);
    
    //functions to use the robotic arm 
    void open()     {    _grip.write(OPEN);     };
    void close()    {    _grip.write(CLOSED);     };
    void toFront()  {    _rotat.write(FRONT);     };
    void toPlate()  {    _rotat.write(PLATE);     };

    //Set the arm to a default position
    void toDefault();

    //functions to control the arm manually for debug or demo purpose
    void gripperWrite(uint8_t pos)  { _grip.write(pos);     };
    void armWrite(uint8_t pos)      { _rotat.write(pos);    };
};

class PlateHolder
{
  private:
    
    static const uint8_t OPEN = 170;
    static const uint8_t CLOSED = 80;

    ServoAct _servo;
  public:

    PlateHolder(uint8_t servoPin);
    
    //functions to use the robotic arm 
    void open()     {    _servo.write(OPEN);     };
    void close()    {    _servo.write(CLOSED);     };

    //Set the plateholder to a default position
    void toDefault()  {    _servo.writeDefault();     };

    //functions to control the arm manually for debug or demo purpose
    void write(uint8_t pos) { _servo.write(pos);    };
};