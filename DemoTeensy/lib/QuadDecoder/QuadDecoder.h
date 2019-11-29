#ifndef QUADDECODER_H
#define QUADDECODER_H

/*  QuadEncoder.h Library was created for the Robotics Systems Lab at RIT
*   By Anmol Modur
*   9/20/19
*   
*   Encoder 1 Pins 

						XBAR1 -> XBARA1
GPIO	PHASE	PAD	IOMUXREG	XBARPORT IOMUX	XBAR1-INPUT	XBAR1-OUTPUT	SELn	n
6		B0_10		
5	ENC2B	EMC_08		ALT3	XBAR_INOUT17	XBAR1_IN17	XBAR1_OUT72	    72	17
3	ENC2A	EMC_05		ALT3	XBAR_INOUT07	XBAR1_IN07	XBAR1_OUT71 	71	07
2	ENC1B	EMC_04		ALT3	XBAR_INOUT06 	XBAR1_IN06	XBAR1_OUT67 	67	06
1	ENC1A	AD_B0_02	ALT1	XBAR_INOUT16	XBAR1_IN16	XBAR1_OUT66 	66	16
*/


// 24, 1

#include <Arduino.h>

#define XBARA1_SEL33 (IMXRT_XBARA1.offset042)
#define XBARA1_SEL35 (IMXRT_XBARA1.offset046)
#define XBARA1_SEL36 (IMXRT_XBARA1.offset048)

//QuadDecoder class with N = 1 or 2
class QuadDecoder
{
private:
    int mode = 0;
    // 0 - Position Control Mode
    // 1 - Velocity Control Mode
    unsigned int XBARA_IN;
    unsigned int XBARB_OUT;
    void XBAR_CONNECT(unsigned int, unsigned int);
    void XBAR_INIT();
    void setENC_CTRL(unsigned int);
    void ENC_INIT(uint32_t);
    void IOMUXC_INIT();
    void PIT_INIT(uint32_t us = 100000);
    int PTRIGGERus = 0;
    int VCPR = 0;
    int N;

public:
    QuadDecoder(int decoderNumber, int OpMode = 0);
    void begin(uint32_t CPR, uint32_t us = 100000);
    uint32_t getCount();
    uint32_t getDCount();
    double getVelocity();
};

#endif