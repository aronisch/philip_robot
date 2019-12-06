#include "ultrasonicSensor.h"
#include "Arduino.h"

UltrasonicSensor::UltrasonicSensor(int echoPin, int trigPin) : _echoPin(echoPin), _trigPin(trigPin){
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}

int UltrasonicSensor::getDist(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
  return distance;
}