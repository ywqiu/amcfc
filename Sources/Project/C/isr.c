#include "ppm.h"
#include "gpio.h"

extern const int8 channel_port[9];
extern int32 ppm_input[9];



int32 int_count=0;

extern int32 send_flag;




void pit0_isr(void)
{    

  DisableInterrupts;	                  //¹Ø×ÜÖÐ¶Ï 
 
   
  int_count++;
  if(int_count>=0)
  {
    int_count=0;
    send_flag=1;
  }
  
  
  
  PIT_TFLG(0)|=PIT_TFLG_TIF_MASK;
  enable_pit_interrupt(0);
  EnableInterrupts;
}


void int_portb_isr()
{
    DisableInterrupts; 
  
    ppm_read();
    
    EnableInterrupts;	
}