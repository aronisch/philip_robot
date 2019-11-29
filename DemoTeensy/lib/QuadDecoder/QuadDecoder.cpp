#include "QuadDecoder.h"


QuadDecoder::QuadDecoder(int decoderNumber, int OpMode)
{
    N = decoderNumber;
    mode = OpMode;
    IOMUXC_INIT();
    XBAR_INIT();
}

void QuadDecoder::begin(uint32_t CPR, uint32_t us)
{   
    VCPR = CPR;
    PTRIGGERus = us;
    ENC_INIT(CPR);
    PIT_INIT(us); 
    
}

uint32_t QuadDecoder::getCount()
{
    // Want unheld position when in position mode but held position in velocity mode
    int N2 = (mode == 0 ? N + 4 : N);
    switch (N2)
    {
    case 1:
        return (ENC1_UPOSH << 16) + (ENC1_LPOSH);
    case 2:
        return (ENC2_UPOSH << 16) + (ENC2_LPOSH);
    case 3:
        return (ENC3_UPOSH << 16) + (ENC3_LPOSH);
    case 4:
        return (ENC4_UPOSH << 16) + (ENC4_LPOSH);
    case 5:
        return (ENC1_UPOS << 16) + (ENC1_LPOS);
    case 6:
        return (ENC2_UPOS << 16) + (ENC2_LPOS);
    case 7:
        return (ENC3_UPOS << 16) + (ENC3_LPOS);
    case 8:
        return (ENC4_UPOS << 16) + (ENC4_LPOS);
    default:
        return 0;
    }
}

uint32_t QuadDecoder::getDCount()
{
    switch (N)
    {
    case 1:
        return (ENC1_POSDH << 16) + (ENC1_POSDH);
    case 2:
        return (ENC2_POSDH << 16) + (ENC2_POSDH);
    case 3:
        return (ENC3_POSDH << 16) + (ENC3_POSDH);
    case 4:
        return (ENC4_POSDH << 16) + (ENC4_POSDH);
    default:
        return 0;
    }
}

double QuadDecoder::getVelocity() // RPM
{
    if (mode == 0)
        return 0;
    return double(getDCount()/VCPR) * double(60000000.0 / PTRIGGERus);
}

void QuadDecoder::XBAR_CONNECT(unsigned int input, unsigned int output)
{
    if (input >= 88)
        return;
    if (output >= 132)
        return;
    volatile uint16_t *xbar = &XBARA1_SEL0 + (output / 2);
    uint16_t val = *xbar;
    if (!(output & 1))
    {
        val = (val & 0xFF00) | input;
    }
    else
    {
        val = (val & 0x00FF) | (input << 8);
    }
    *xbar = val;
}

void QuadDecoder::XBAR_INIT()
{
    CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON); // XBAR CLK ON
    switch (N)
    {
    case 1:
        XBAR_CONNECT(16, 66);
        XBAR_CONNECT(6, 67);
        break;
    case 2:
        XBAR_CONNECT(7, 71);
        XBAR_CONNECT(17, 72);
        break;
    default:
        return;
    }
}

void QuadDecoder::setENC_CTRL(unsigned int value)
{
    uint16_t ENCN_CTRL = ENC1_CTRL + ((N - 1) * 0x4000);
    volatile uint16_t *enc1 = &ENCN_CTRL;
    uint16_t val = *enc1;
    val = val | value;
    *enc1 = val;
}

void QuadDecoder::ENC_INIT(uint32_t CPR)
{
    uint16_t LMOD = (CPR & 0x0000FFFF);
    uint16_t UMOD = (CPR & 0xFFFF0000) >> 16;
    switch (N)
    {
    case 1:
        CCM_CCGR4 |= CCM_CCGR4_ENC1(CCM_CCGR_ON);
        ENC1_CTRL &= ~(0xFFFF);
        ENC1_CTRL |= 0x0404;
        ENC1_CTRL2 &= ~(0xFFFF);
        ENC1_CTRL2 |= (mode == 0 ? 0x0104 : 0x0105);
        ENC1_LMOD |= LMOD;
        ENC1_UMOD |= UMOD;
        break;
    case 2:
        CCM_CCGR4 |= CCM_CCGR4_ENC2(CCM_CCGR_ON);
        ENC2_CTRL &= ~(0xFFFF);
        ENC2_CTRL |= 0x0404;
        ENC2_CTRL2 &= ~(0xFFFF);
        ENC2_CTRL2 |= (mode == 0 ? 0x0104 : 0x0105);
        ENC2_LMOD |= LMOD;
        ENC2_UMOD |= UMOD;
        break;
    case 3:
        CCM_CCGR4 |= CCM_CCGR4_ENC3(CCM_CCGR_ON);
        ENC3_CTRL &= ~(0xFFFF);
        ENC3_CTRL |= 0x0404;
        ENC3_CTRL2 &= ~(0xFFFF);
        ENC3_CTRL2 |= (mode == 0 ? 0x0104 : 0x0105);
        ENC3_LMOD |= LMOD;
        ENC3_UMOD |= UMOD;
        break;
    case 4:
        CCM_CCGR4 |= CCM_CCGR4_ENC4(CCM_CCGR_ON);
        ENC4_CTRL &= ~(0xFFFF);
        ENC4_CTRL |= 0x0404;
        ENC4_CTRL2 &= ~(0xFFFF);
        ENC4_CTRL2 |= (mode == 0 ? 0x0104 : 0x0105);
        ENC4_LMOD |= LMOD;
        ENC4_UMOD |= UMOD;
        break;
    default:
        break;
    }
}

void QuadDecoder::IOMUXC_INIT()
{
    switch (N)
    {
    case 1:
        IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 &= ~(0x00000007);
        IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B0_02 |= 0x1;
        IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 &= ~(0x00000007);
        IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_04 |= 0x3;
        break;
    case 2:
        IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 &= ~(0x00000007);
        IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_05 |= 0x3;
        IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 &= ~(0x00000007);
        IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 |= 0x3;
        break;
    default:
        return;
    }
}

void QuadDecoder::PIT_INIT(uint32_t us)
{
    CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);
    PIT_MCR = 0x00; // Turn On Timer
    uint32_t cycles = (24000000 / 1000000) * us - 1;
    switch (N)
    {
    case 1:
        XBAR_CONNECT(56, 70);
        PIT_TCTRL0 &= ~(0x1);
        PIT_LDVAL0 = cycles;
        PIT_TCTRL0 |= 1;
        break;
    case 2:
        XBAR_CONNECT(57, 75);
        PIT_TCTRL1 &= ~(0x1);
        PIT_LDVAL1 = cycles;
        PIT_TCTRL1 |= 1;
        break;
    case 3:
        XBAR_CONNECT(58, 80);
        PIT_TCTRL2 &= ~(0x1);
        PIT_LDVAL2 = cycles;
        PIT_TCTRL2 |= 1;
        break;
    case 4:
        XBAR_CONNECT(59, 85);
        PIT_TCTRL3 &= ~(0x1);
        PIT_LDVAL3 = cycles;
        PIT_TCTRL3 |= 1;
        break;
    default:
        break;
    }
}