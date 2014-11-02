#include "includes.h"              

extern uint8 flight_status;

void light_init(void)
{
    gpio_init(PORTA,4,1,1);                    //工作指示灯初始化
    gpio_init(PORTA,5,1,1);                        //告警指示灯初始化
    
    gpio_init(PORTB,18,1,1);
    
    gpio_init(PORTB,19,1,1);
}


void light_flash(void)
{
    light_runtime(flight_status);
    light_warning(0);
}
    
void light_ctrl(uint8 light,uint8 cmd)
{
    switch(light)
    {
        case RUNLIGHT:gpio_ctrl(PORTA,4,cmd);break;
        case WARNLIGHT:gpio_ctrl(PORTA,5,cmd);break;
        default:break;
    }
}

void light_runtime(uint8 cmd)
{
  const uint8 light[4][40]={  {1,1,1,1,1,1,1,1,0,0,1,1,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},    //RUN_LOCK
                              {1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0,1,1,0,0},    //RUN_MORMAL
                              {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                              {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}}  ;
    static uint8 count=0;
    
    
    count++;
    if(count>=40)
        count=0;
    
    gpio_ctrl(PORTA,4,light[cmd][count]);
}


void light_warning(uint8 cmd)
{
    const uint8 light[5][40]={{1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},    //RUN_OK,warnning off
                              {1,1,1,1,1,0,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},    //
                              {1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1,1,1,1,0,0,0,0,1},
                              {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
                              {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}}  ;
    static uint8 count=0;
    
    
    count++;
    if(count>=40)
        count=0;
    
    gpio_ctrl(PORTA,5,light[cmd][count]);
}