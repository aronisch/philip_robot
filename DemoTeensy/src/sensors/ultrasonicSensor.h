#ifndef __ULTRASONICSENSOR_H__
#define __ULTRASONICSENSOR_H__

class TrigHandler {

  private :
   
   int m_enable;
   int m_nReady;
   int m_trigP;
   bool m_ready;
   

  public : 
    
  TrigHandler(int Trig);
  void enabler();
  void Ready();
  bool getReady();
  void trig();
  void disabler();

};

class EchoHandler {

  private :

    int m_echoP;
    unsigned long m_startTime;
    unsigned long m_endTime;
    bool m_flag;
    static void echoInterrupt_11();
    static void echoInterrupt_12(); 
    int m_enable; 

  public :

    EchoHandler(int echo);
    unsigned long getEndTime();
    unsigned long getStartTime();
    bool getflag();
    int getPin();
    void enabler();
    void setflag(bool b);
};


class UltrasonicSensor {
  private :
    unsigned int m_mesure;
    bool m_dejaTest;
    int m_a;

    EchoHandler* echohandler;
    TrigHandler* trighandler;


  public :
    void attach(int trig, int echo);
    void update();
    unsigned int getMesure();
    void trig();
    bool getReady();
};

extern EchoHandler echohandler11;
extern EchoHandler echohandler12;

extern TrigHandler trighandler0;
extern TrigHandler trighandler10;


#endif