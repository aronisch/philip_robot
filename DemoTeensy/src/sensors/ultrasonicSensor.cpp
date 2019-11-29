#include "ultrasonicSensor.h"
#include <Arduino.h>
#define TEMPORISATION 50000


EchoHandler echohandler11(11);
EchoHandler echohandler12(12);

TrigHandler trighandler0(0);
TrigHandler trighandler10(10);


EchoHandler::EchoHandler(int echo)
  : m_echoP(echo) {
  m_enable = 0;
  m_startTime = 0;
  m_endTime = 0;
}

unsigned long EchoHandler::getEndTime() {
  return m_endTime;
}

unsigned long EchoHandler::getStartTime() {
  return m_startTime;
}

bool EchoHandler::getflag() {
  return m_flag;
}

int EchoHandler::getPin() {
  return m_echoP;
}

void EchoHandler::enabler() {
    m_enable +=1;
    pinMode(m_echoP, INPUT);
    if (m_echoP == 11) {
      attachInterrupt(digitalPinToInterrupt(11), EchoHandler::echoInterrupt_11, CHANGE);
    }
    if (m_echoP == 12) {
      attachInterrupt(digitalPinToInterrupt(12), EchoHandler::echoInterrupt_12, CHANGE);
  }
}

void EchoHandler::echoInterrupt_11() {
  if (digitalRead(11) == HIGH)
  {
    echohandler11.m_startTime = micros();
  }
  else if (digitalRead(11) == LOW)
  {
    echohandler11.m_endTime = micros();
    echohandler11.m_flag = true;
  }
}

void EchoHandler::echoInterrupt_12() {
  if (digitalRead(12) == HIGH)
  {
    echohandler12.m_startTime = micros();
  }
  else if (digitalRead(12) == LOW)
  {
    echohandler12.m_endTime = micros();
    echohandler12.m_flag = true;
  }
}

void EchoHandler::setflag(bool b){
  m_flag = b;
}

TrigHandler::TrigHandler(int Trig)
  : m_trigP(Trig)
{
  m_enable = 0;
  m_nReady = 0;
  m_ready = true;
}

void TrigHandler::trig() {
  if (m_ready) {
    digitalWrite(m_trigP, HIGH);
    delayMicroseconds(20);
    digitalWrite(m_trigP, LOW);
    m_ready = false;
    m_nReady = 0;
  }
}

void TrigHandler::enabler() {
  m_enable += 1;
  pinMode(m_trigP, OUTPUT);
}

void TrigHandler::disabler() {
  if (m_enable != 0) {
    m_enable -= 1;
  }
}


void TrigHandler::Ready() {
  m_nReady += 1;
  if (m_nReady >= m_enable) {
    m_ready = true;
  }
}

bool TrigHandler::getReady() {
  return m_ready;
}


bool UltrasonicSensor::getReady() {
  return trighandler->getReady();
}

void UltrasonicSensor::trig() {
  trighandler->trig();
}

void UltrasonicSensor::update() {
  if (echohandler->getflag()) {
    m_mesure = (echohandler->getEndTime() - echohandler->getStartTime()) * 340 / 2000;
    echohandler->setflag(false);
    m_dejaTest = false;
  }

  else if (((micros() - echohandler->getEndTime()) >= TEMPORISATION) && !m_dejaTest) {
    m_dejaTest = true;
    trighandler->Ready();
  }
}



unsigned int UltrasonicSensor::getMesure() {
  return m_mesure;
}


void UltrasonicSensor::attach(int trig, int echo)
{
  switch (trig)
  {
  case 0:
    trighandler = &trighandler0; break;
  case 10:
    trighandler = &trighandler10; break;
  default:
    trighandler = NULL; break;
  }
  
  switch (echo){
  case 11:
    echohandler = &echohandler11; break;
  case 12:
    echohandler = &echohandler12; break;
  default: 
    echohandler = NULL; break;
  }

  m_a = 0;
  echohandler->enabler();
  trighandler->enabler();
}