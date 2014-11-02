#ifndef _COMMON_H_
#define _COMMON_H_


    #include "MK60N512VMD100.h"               //registers definitions
    
    #define ARM_INTERRUPT_LEVEL_BITS     4    //interrupt priority
    #define EnableInterrupts asm(" CPSIE i"); //enable interrupts
    #define DisableInterrupts asm(" CPSID i");//disable interrupts
    
    
    //data type definitions
    typedef unsigned char	uint8;    //  8 bits
    typedef unsigned short int	uint16;   // 16 bits
    typedef unsigned long int	uint32;   // 32 bits
    
    typedef char		int8;     //  8 bits
    typedef short int	       	int16;    // 16 bits
    typedef int		       	int32;    // 32 bits
    
    typedef volatile int8	vint8;    //  8 bits 
    typedef volatile int16	vint16;   // 16 bits
    typedef volatile int32	vint32;   // 32 bits
    
    typedef volatile uint8	vuint8;   //  8 bits
    typedef volatile uint16	vuint16;  // 16 bits
    typedef volatile uint32	vuint32;  // 32 bits

    
    void stop (void);   //CPU to stop
    
    void wait (void);   //CPU to wait
    
    void write_vtor (int);  //modify interrupt vectors table bias registers
    
    void enable_irq (int);  //enable irq th interrupt
   
    void disable_irq (int); //disable irq th interrupt
    
    void set_irq_priority (int, int);   //set priority of irq th interrupt
    
    void main(void);
    
#endif 
