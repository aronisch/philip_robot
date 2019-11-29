#include <Arduino.h>
#include "sensors/frontSensors.h"
#include "pin.h"

FrontSensors frontUS = FrontSensors(US_TRIG_LEFT, US_ECHO_LEFT, US_TRIG_RIGHT, US_ECHO_RIGHT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  frontUS.start();
}

void loop() {
  // put your main code here, to run repeatedly:
  frontUS.newMeasurement();
  Serial.print("L : ");Serial.print(frontUS.getLeftDist());Serial.print("mm   ");
  Serial.print("R : ");Serial.print(frontUS.getRightDist());Serial.print("mm   ");
  Serial.println();
  delay(500);
}