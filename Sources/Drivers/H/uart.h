#ifndef __UART_H__
#define __UART_H__
	

    #include "common.h"
    
    #define UART0 UART0_BASE_PTR
    #define UART1 UART1_BASE_PTR
    #define UART2 UART2_BASE_PTR
    #define UART3 UART3_BASE_PTR
    #define UART4 UART4_BASE_PTR
    #define UART5 UART5_BASE_PTR

    #define UART0irq 45
    #define UART1irq 47
    #define UART2irq 49
    #define UART3irq 51
    #define UART4irq 53
    #define UART5irq 55


    void uart_init (UART_MemMapPtr uartch, uint32 sysclk, uint32 baud);
       
    uint8 uart_re1(UART_MemMapPtr uartch,uint8 *ch);
    
    void uart_send1(UART_MemMapPtr uartch, uint8 ch);
    
    uint8  uart_reN (UART_MemMapPtr uartch ,uint8* buff,uint16 len);
        
    void uart_sendN (UART_MemMapPtr uartch ,uint8* buff,uint16 len);

    //enable UART interrupt, irqno: irq number of interruppt
    void enableuartreint(UART_MemMapPtr uartch,uint8 irqno);
    
    void disableuartreint(UART_MemMapPtr uartch,uint8 irqno);
    
    void uart_sendstring (UART_MemMapPtr uartch ,uint8* buff);
   
    uint8 uart_sendnumber(UART_MemMapPtr uartch , int number);

    
#endif 
