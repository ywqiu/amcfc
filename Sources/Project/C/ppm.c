#include "includes.h"

extern uint8 flight_status;
extern uint8 flight_mode;
extern uint8 light_OnOff;

extern uint8 channel_port[9];
extern int32 ppm_input[9];
extern int32 ppmADC[9];
extern int32 ppm_error;

extern int16 accCAL[3],gyroCAL[3]; 

int32 ppm_time_l[9],ppm_time_h[9];


//initialize rc ppm input;
void ppm_init(void)
{
    uint8 i;
    for(i=0;i<=8;i++)
    {
        exti_init(PORTB,channel_port[i],either_up,2);
    }
}


void ppm_read(void)
{
    for(int i=0;i<=0x2f;i++);
  
    uint32 PORTB_PDIR_DATA=GPIO_PDIR_REG(PORTB);
    uint32 PORTB_ISFR_DATA=PORTB_ISFR;
    int32 temp;
      
    for(uint8 i=0;i<=8;i++)
    {
        if(PORTB_ISFR_DATA & (1 << channel_port[i]))
        {
            if(PORTB_PDIR_DATA & (1 << channel_port[i])) // 若为上升沿
                ppm_time_h[i]=PIT_CVAL0;
            else    // 若为下降沿
            {
                ppm_time_l[i]=PIT_CVAL0;
                temp=ppm_time_h[i]-ppm_time_l[i];
                while(temp<0)
                    temp+=125000;
                //if((temp>=50000)&&(temp<=100000))
                    ppm_input[i]=temp;
                //else
                //    ppm_error++;
            }
            PORTB_ISFR  |= (1 << channel_port[i]);
        }
    }
}

void ppm_transform(void)
{ 
    static int32 ppmPRE[9];  
    uint8 u;
    
    for(u=0;u<=8;u++)
        ppmADC[u]=(ppm_input[u]-50000)/25;
    
   //limit rc rise rate 
   for(uint8 u=0;u<=8;u++)
   {   
      if(ppmADC[u]<ppmPRE[u]-RC_LIMIT)
          ppmADC[u]=ppmPRE[u]-RC_LIMIT;
      if(ppmADC[u]>ppmPRE[u]+RC_LIMIT)
          ppmADC[u]=ppmPRE[u]+RC_LIMIT;
     
      ppmPRE[u]=ppmADC[u];
     
   }
    
    if(ppmADC[FMOD]<500)
        flight_mode=0;
    else if(ppmADC[FMOD]<1500)
        flight_mode=1;
    else
        flight_mode=2;
    
    if(ppmADC[AUX2]<1000)
        light_OnOff=0;
    else
        light_OnOff=1;
    
}

uint8 ppm_gesture(void)
{
    static uint8 count=0;
    static uint8 temp=0;
    
    if(temp==GES_WAIT)
    {
        if((ppmADC[THRO]<200)&&(ppmADC[RUDD]>1800))
            temp=GES_UNLOCK;
        
        if((ppmADC[THRO]<200)&&(ppmADC[RUDD]<200))
            temp=GES_LOCK;
        
        if((flight_status==LOCK)&&(ppmADC[THRO]>1800)&&(ppmADC[RUDD]<200)&&(ppmADC[ELEV]<200))
            temp=GES_CALI;
    }
    
    if(temp==GES_UNLOCK)
    {
        if((ppmADC[THRO]<200)&&(ppmADC[RUDD]>1800))
        {
            if(count>=30)
                flight_status=NORMAL;
            else
                count++;
        }
        else
        {
            count=0;
            temp=GES_WAIT;
        }
    }
    
    if(temp==GES_LOCK)
    {
        if((ppmADC[THRO]<200)&&(ppmADC[RUDD]<200))
        {
            if(count>=30)
                flight_status=LOCK;
            else
                count++;
        }
        else
        {
            count=0;
            temp=GES_WAIT;
        }
    }
    
    if(temp==GES_CALI)
    {
        if((ppmADC[THRO]>1800)&&(ppmADC[RUDD]<200)&&(ppmADC[ELEV]<200))
        {
            if(count>=40)
                mpu6050_calibration(gyroCAL,accCAL);/***********************/
            else
                count++;
        }
        else
        {
            count=0;
            temp=GES_WAIT;
        }
    }
}