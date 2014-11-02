#ifndef __GPIO_H__
#define __GPIO_H__
    

    #include "common.h"
    
    #define PORTA PTA_BASE_PTR
    #define PORTB PTB_BASE_PTR
    #define PORTC PTC_BASE_PTR
    #define PORTD PTD_BASE_PTR
    #define PORTE PTE_BASE_PTR
     
    typedef enum exti_cfg
    {
        zero_down     = 0x08u,     //�͵�ƽ�������ڲ�����
        rising_down   = 0x09u,     //�����ش������ڲ�����
        falling_down  = 0x0Au,     //�½��ش������ڲ�����
        either_down   = 0x0Bu,     //�����ش������ڲ�����
        one_down      = 0x0Cu,     //�ߵ�ƽ�������ڲ�����
    
        zero_up       = 0x88u,     //�͵�ƽ�������ڲ�����
        rising_up     = 0x89u,     //�����ش������ڲ�����
        falling_up    = 0x8Au,     //�½��ش������ڲ�����
        either_up     = 0x8Bu,     //�����ش������ڲ�����
        one_up        = 0x8Cu      //�ߵ�ƽ�������ڲ�����
    } exti_cfg;
   
  
    void gpio_init (GPIO_MemMapPtr port, int index, int dir,int data);
    
    void gpio_ctrl (GPIO_MemMapPtr port, int index, int data);
    
    void gpio_reverse (GPIO_MemMapPtr port, int index);

    void exti_init(GPIO_MemMapPtr port, int index, exti_cfg cfg, int priority);
    
#endif 
