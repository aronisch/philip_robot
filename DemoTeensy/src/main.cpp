#include <Arduino.h>
#include "pin.h"
#include "actuators/actuators.h"
#include "MotorControl/Motor.h"
#include "MotorControl/Odometry.h"
#include "MotorControl/DifferentialDrive.h"
#include "sensors/frontSensors.h"
#include "pi_communication/SerialRPi.h"

#define UPDATE_TIMER_PERIOD (10000)
#define TICKS_PER_REVOLUTION (2249)
#define WHEEL_RADIUS (34)
#define AXLE_DISTANCE (270)

void robotControl();
void componentTesting();

FrontSensors frontUS = FrontSensors(US_TRIG_LEFT, US_ECHO_LEFT, US_TRIG_RIGHT, US_ECHO_RIGHT);

IntervalTimer updateTimer;

Odometry *odo;
Motor *motorRight;
Motor *motorLeft;
MotorPID *motorControl;
DifferentialDrive *diffDrive;
SerialRPi *rpi;
SimpleArm arm = SimpleArm(SERVO_GRIP, SERVO_GRIP_ROTATION);
PlateHolder plateH = PlateHolder(SERVO_PLATE);

bool odoPrint = false;
bool speedPrint = false;
bool pidEnabled = false;

void updateFunction(){
  odo->updateSpeedsAndPositions();
  if(pidEnabled)
    motorControl->speedControlLoop(odo->getSpeedRight(), odo->getSpeedLeft());
}

void setup() {
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, HIGH);

  frontUS.start();

  motorRight = new Motor(MOTOR_PWM_2, MOTOR_DIR_2, false);
  motorLeft = new Motor(MOTOR_PWM_1, MOTOR_DIR_1, true);
  odo = new Odometry(TICKS_PER_REVOLUTION, WHEEL_RADIUS, AXLE_DISTANCE, UPDATE_TIMER_PERIOD);
  motorControl = new MotorPID(motorLeft, motorRight, UPDATE_TIMER_PERIOD, 2.5, 1.2, 0.2, 2.25, 1.06, 0.2);
  diffDrive = new DifferentialDrive(motorControl, AXLE_DISTANCE);
  
  Serial2.begin(9600);
  rpi = new SerialRPi(&Serial2, diffDrive, &arm, &plateH, &frontUS);
  updateTimer.begin(updateFunction, UPDATE_TIMER_PERIOD);

  Serial.begin(9600);
  Serial.println("R Right / L Left / O Odometry / S Speed / P Pid Enable Disable / A and B Open Loop / U Ultrasonic");
  pidEnabled = true;
}

void loop() {
  //componentTesting();
  robotControl();
}

void robotControl(){
  rpi->update();
}

void componentTesting(void){
    while(Serial.available()){
    switch (Serial.read())
    {
      case 'R':
      {
        Serial.println("R - Enter number:");
        while(!Serial.available());
        int speed = Serial.parseInt();
        Serial.println(speed);
        motorControl->setSpeedRight(speed);
        break;
      }
      case 'L':
      {
        Serial.println("L - Enter number:");
        while(!Serial.available());
        int speed = Serial.parseInt();
        Serial.println(speed);
        motorControl->setSpeedLeft(speed);
        break;
      }
      case 'A':
      {
        Serial.println("Right Open Loop - Enter number:");
        while(!Serial.available());
        int speed = Serial.parseInt();
        Serial.println(speed);
        motorRight->setPWMSpeed(speed);
        break;
      }
      case 'B':
      {
        Serial.println("Left Open Loop - Enter number:");
        while(!Serial.available());
        int speed = Serial.parseInt();
        Serial.println(speed);
        motorLeft->setPWMSpeed(speed);
        break;
      }
      case 'O':
      {
        odoPrint = !odoPrint;
        break;
      }
      case 'S':
      {
        speedPrint = !speedPrint;
        break;
      }
      case 'P':
      {
        pidEnabled = !pidEnabled;
        Serial.print("PID :");
        Serial.println(pidEnabled);
        break;
      }
      case 'U':
      {
        frontUS.newMeasurement();
        Serial.print("L : ");Serial.print(frontUS.getLeftDist());Serial.print("mm   ");
        Serial.print("R : ");Serial.print(frontUS.getRightDist());Serial.print("mm   ");
        Serial.println();
      }
      case 'F':
      {
        odo->resetPosition();
        pidEnabled = 1;

        diffDrive->setVelocities(500, 0);
        delay(1500);

        diffDrive->setVelocities(0, 0);

        Serial.print("X: ");
        Serial.print(odo->getPositionX());
        Serial.print(" - Y: ");
        Serial.print(odo->getPositionY());
        Serial.print(" - Theta: ");
        Serial.println(odo->getPositionTheta());
        delay(1500);

        diffDrive->setVelocities(-500, 0);

        delay(1500);

        diffDrive->setVelocities(0, 0);

        Serial.print("X: ");
        Serial.print(odo->getPositionX());
        Serial.print(" - Y: ");
        Serial.print(odo->getPositionY());
        Serial.print(" - Theta: ");
        Serial.println(odo->getPositionTheta());
      }
      case 'T':
      {
        odo->resetPosition();
        pidEnabled = 1;

        diffDrive->setVelocities(0, 3);
        delay(1500);

        diffDrive->setVelocities(0, 0);

        Serial.print("X: ");
        Serial.print(odo->getPositionX());
        Serial.print(" - Y: ");
        Serial.print(odo->getPositionY());
        Serial.print(" - Theta: ");
        Serial.println(odo->getPositionTheta());
        delay(1500);

        diffDrive->setVelocities(0, -3);

        delay(1500);

        diffDrive->setVelocities(0, 0);

        Serial.print("X: ");
        Serial.print(odo->getPositionX());
        Serial.print(" - Y: ");
        Serial.print(odo->getPositionY());
        Serial.print(" - Theta: ");
        Serial.println(odo->getPositionTheta());
      }
    }
  }
  if(odoPrint){
    Serial.print("X: ");
    Serial.print(odo->getPositionX());
    Serial.print(" - Y: ");
    Serial.print(odo->getPositionY());
    Serial.print(" - Theta: ");
    Serial.println(odo->getPositionTheta());
  }
  if(speedPrint){
    Serial.print("Speed R: ");
    Serial.print(odo->getSpeedRight());
    Serial.print(" - Speed L: ");
    Serial.println(odo->getSpeedLeft());
  }
  delay(10);
}