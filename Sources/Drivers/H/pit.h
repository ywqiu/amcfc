#ifndef PIT_H_
#define PIT_H_
   
    #include "common.h"
    
    #define PIT0 0
    #define PIT1 1
    #define PIT2 2
    #define PIT3 3
    
  
    void pit_init(uint8 pitno,uint32 timeout);
     
    void enable_pit_interrupt(uint8 pitno);
    
    void disable_pit_interrupt(uint8 pitno);
  
  
#endif 