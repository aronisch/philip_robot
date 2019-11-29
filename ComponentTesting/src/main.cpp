#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "pin.h"
#include "QuadDecoder.h"
#include "../lib/Adafruit_VL53L0X-master/src/Adafruit_VL53L0X.h"

#define TICKS_PER_REV 2249 // Number of ticks per wheel revolution 48*46.85
#define SPEED_CALCULATION_PERIOD 10000       // How fast velocity is calculated in microseconds

const double rpmConvert = 60000000 / (SPEED_CALCULATION_PERIOD * TICKS_PER_REV);

QuadDecoder Enc1 = QuadDecoder(1);
QuadDecoder Enc2 = QuadDecoder(2);

Adafruit_VL53L0X lidar1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lidar2 = Adafruit_VL53L0X();

IntervalTimer velTimer;

Servo servoPlate;
Servo servoGripRotation;
Servo servoGrip;

volatile int oldCntA = 0; // used to store the count the last time you calculated
volatile int newCntA = 0; //  new value to use in calculating difference
volatile int diffA = 0;   // difference between newCntA and oldCntA

volatile double motorRPMA = 0.0;

volatile int oldCntB = 0; // used to store the count the last time you calculated
volatile int newCntB = 0; //  new value to use in calculating difference
volatile int diffB = 0;   // difference between newCntA and oldCntA

volatile double motorRPMB = 0.0;

void calcVel(void)
{
    newCntA = Enc2.getCount();
    diffA = newCntA - oldCntA;
    oldCntA = newCntA;
    motorRPMA = (double)diffA * rpmConvert;

    newCntB = Enc1.getCount();
    diffB = newCntB - oldCntB;
    oldCntB = newCntB;
    motorRPMB = (double)diffB * rpmConvert;
}

void setup() {
  pinMode(MOTOR_PWM_1, OUTPUT);
  pinMode(MOTOR_PWM_2, OUTPUT);
  pinMode(MOTOR_DIR_1, OUTPUT);
  pinMode(MOTOR_DIR_2, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);

  digitalWrite(MOTOR_ENABLE, HIGH);
  digitalWrite(MOTOR_DIR_1, LOW);
  digitalWrite(MOTOR_DIR_2, HIGH);

  pinMode(US_TRIG_LEFT, OUTPUT);
  pinMode(US_TRIG_RIGHT, OUTPUT);
  pinMode(US_ECHO_LEFT, INPUT);
  pinMode(US_ECHO_RIGHT, INPUT);

  Enc1.begin(TICKS_PER_REV);
  Enc2.begin(TICKS_PER_REV);

  velTimer.begin(calcVel, SPEED_CALCULATION_PERIOD);

  servoPlate.attach(SERVO_PLATE);
  servoGripRotation.attach(SERVO_GRIP_ROTATION);
  servoGrip.attach(SERVO_GRIP);

  Serial.begin(9600);
  Serial.println("M Motor Speed / P Plate / R Rotation / G Gripper / S Speed");

  lidar1.begin(VL53L0X_I2C_ADDR, true);
  lidar2.begin(VL53L0X_I2C_ADDR, true, &Wire1);
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure1, measure2;
  while(Serial.available()){
    switch (Serial.read())
    {
    case 'M':
    {
      Serial.println("Enter number:");
      while(!Serial.available());
      int speed = Serial.parseInt();
      Serial.println(speed);
      analogWrite(MOTOR_PWM_1, speed);
      analogWrite(MOTOR_PWM_2, speed);
      break;
    }
    case 'P':
    {
      Serial.println("Enter number:");
      while(!Serial.available());
      int pos = Serial.parseInt();
      Serial.println(pos);
      servoPlate.write(pos);
      break;
    }
    case 'R':
    {
      Serial.println("Enter number:");
      while(!Serial.available());
      int pos = Serial.parseInt();
      Serial.println(pos);
      servoGripRotation.write(pos);
      break;
    }
    case 'G':
    {
      Serial.println("Enter number:");
      while(!Serial.available());
      int pos = Serial.parseInt();
      Serial.println(pos);
      servoGrip.write(pos);
      break;
    }
    case 'S':
    {
      Serial.print("Speed A: ");
      Serial.print(motorRPMA);
      Serial.print(" - Speed B: ");
      Serial.println(motorRPMB);
      break;
    }
    case 'L':
    {
      lidar1.rangingTest(&measure1, true);
      lidar2.rangingTest(&measure2, true);
      if (measure1.RangeStatus != 4) {  // phase failures have incorrect data
        Serial.print("Lidar 1: ");
        Serial.println(measure1.RangeMilliMeter);
      } else {
        Serial.println("Lidar 1: out of range ");
      }
      if (measure2.RangeStatus != 4) {  // phase failures have incorrect data
        Serial.print("Lidar 2: ");
        Serial.println(measure2.RangeMilliMeter);
      } else {
        Serial.println(" out of range ");
      }
      break;
    }
    case 'U':
    {
      long durationLeft, durationRight;
      digitalWrite(US_TRIG_LEFT, LOW);
      delayMicroseconds(5);
      digitalWrite(US_TRIG_LEFT, HIGH);
      delayMicroseconds(10);
      digitalWrite(US_TRIG_LEFT, LOW);

      pinMode(US_ECHO_LEFT, INPUT);
      durationLeft = pulseIn(US_ECHO_LEFT, HIGH);

      digitalWrite(US_TRIG_RIGHT, LOW);
      delayMicroseconds(5);
      digitalWrite(US_TRIG_RIGHT, HIGH);
      delayMicroseconds(10);
      digitalWrite(US_TRIG_RIGHT, LOW);

      pinMode(US_ECHO_RIGHT, INPUT);
      durationRight = pulseIn(US_ECHO_RIGHT, HIGH);

      Serial.print("US Left: ");
      Serial.print((durationLeft/2) / 29.1);
      Serial.print(" - US Right: ");
      Serial.println((durationRight/2) / 29.1);
      break;
    }
  }
}
}