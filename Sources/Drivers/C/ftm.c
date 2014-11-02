#include "ftm.h"


void motor_control(int16 num_motor,int16 pwm)
{
  if(pwm<0)
  {
    pwm=0;
  }
  if(pwm>(int16)FTM0_MOD)
  {
    pwm=FTM0_MOD;
  }
 
  switch(num_motor)
  {
    case 1: FTM0_C5V=pwm;break;
    case 2: FTM0_C4V=pwm;break;
    case 3: FTM0_C2V=pwm;break;
    case 4: FTM0_C1V=pwm;break;
    default: break;
  }
  
  FTM0_CNT=0;
}


void pwm_init(void)
{      	
      //SIM_SOPT4|=SIM_SOPT4_FTM1FLT0_MASK;        
      /* Turn on all port clocks */
      SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;
        
      /* Enable the function on PTA8 */
      PORTC_PCR1 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // motor1      ftm0_ch5   
      PORTC_PCR2 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // motor2      ftm0_ch4
      PORTC_PCR3 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // motor3      ftm0_ch1
      PORTC_PCR4 = PORT_PCR_MUX(0x4)| PORT_PCR_DSE_MASK; // motor4      ftm0_ch0
      
      
      SIM_SCGC6|=SIM_SCGC6_FTM0_MASK;         //使能FTM0时钟      
      
      //change MSnB = 1  
      FTM0_C0SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C0SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C0SC |= FTM_CnSC_MSB_MASK; 
      
      FTM0_C1SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C1SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C1SC |= FTM_CnSC_MSB_MASK;     
      
      FTM0_C2SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C2SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C2SC |= FTM_CnSC_MSB_MASK; 
      
      FTM0_C3SC |= FTM_CnSC_ELSB_MASK;
      FTM0_C3SC &= ~FTM_CnSC_ELSA_MASK;
      FTM0_C3SC |= FTM_CnSC_MSB_MASK; 
      
     
      
      //FTM1_SC = FTM_SC_PS(0) | FTM_SC_CLKS(1);
      //FTM1_SC=0X0F;     
      FTM0_SC = 0x2d; //not enable the interrupt mask
      //FTM1_SC=0X1F;       //BIT5  0 FTM counter operates in up counting mode.
                            //1 FTM counter operates in up-down counting mode.      
      //BIT43 FTM1_SC|=FTM1_SC_CLKS_MASK;
                            //00 No clock selected (This in effect disables the FTM counter.)
                            //01 System clock
                            //10 Fixed frequency clock
                            //11 External clock
      //BIT210 FTM1_SC|=FTM1_SC_PS_MASK; 
                            //100M          MOD=2000;     MOD=4000;   MOD=1000; 
                            //000 Divide by 1---12KHZ     6K          24k
                            //001 Divide by 2--- 6KHZ
                            //010 Divide by 4--- 3K
                            //011 Divide by 8--- 1.5K
                            //100 Divide by 16---750
                            //101 Divide by 32---375
                            //110 Divide by 64---187.5HZ
                            //111 Divide by 128--93.75hz             
      
      FTM0_MODE |= FTM_MODE_WPDIS_MASK;      
       //BIT1   Initialize the Channels Output
      //FTMEN is bit 0, need to set to zero so DECAPEN can be set to 0
      FTM0_MODE &= ~1;
       //BIT0   FTM Enable
       //0 Only the TPM-compatible registers (first set of registers) can be used without any restriction. Do not use the FTM-specific registers.
       //1 All registers including the FTM-specific registers (second set of registers) are available for use with no restrictions.
      
      FTM0_OUTMASK=0X00;   //0 Channel output is not masked. It continues to operate normally.
                           //1 Channel output is masked. It is forced to its inactive state.
      
      FTM0_COMBINE=0;      //Function for Linked Channels (FTMx_COMBINE)
      FTM0_OUTINIT=0;
      FTM0_EXTTRIG=0;      //FTM External Trigger (FTMx_EXTTRIG)
      FTM0_POL=0;          //Channels Polarity (FTMx_POL)
                           //0 The channel polarity is active high.
                           //1 The channel polarity is active low.     
      //Set Edge Aligned PWM
      FTM0_QDCTRL &=~FTM_QDCTRL_QUADEN_MASK;
      //QUADEN is Bit 1, Set Quadrature Decoder Mode (QUADEN) Enable to 0,   (disabled)
      //FTM0_SC = 0x16; //Center Aligned PWM Select = 0, sets FTM Counter to operate in up counting mode,
      //it is field 5 of FTMx_SC (status control) - also setting the pre-scale bits here
      
      FTM0_INVCTRL=0;     //反转控制
      FTM0_SWOCTRL=0;     //软件输出控制F TM Software Output Control (FTMx_SWOCTRL)
      FTM0_PWMLOAD=0;     //FTM PWM Load
                          //BIT9: 0 Loading updated values is disabled.
                          //1 Loading updated values is enabled.
      FTM0_CNTIN=0;        //Counter Initial Value      
      FTM0_MOD=1953;//1953       //Modulo value,The EPWM period is determined by (MOD - CNTIN + 0x0001) 
                           //采用龙丘时钟初始化函数，可以得到4分频的频率，系统60M频率时，PWM频率是15M,以此类推
                           //PMW频率=X系统频率/4/(2^FTM1_SC_PS)/FTM1_MOD
      FTM0_C3V=781;        //设置 the pulse width(duty cycle) is determined by (CnV - CNTIN).
      FTM0_C2V=781;
      FTM0_C1V=781;
      FTM0_C0V=781;
      FTM0_CNT=0;          
}
