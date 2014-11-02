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
        zero_down     = 0x08u,     //低电平触发，内部下拉
        rising_down   = 0x09u,     //上升沿触发，内部下拉
        falling_down  = 0x0Au,     //下降沿触发，内部下拉
        either_down   = 0x0Bu,     //跳变沿触发，内部下拉
        one_down      = 0x0Cu,     //高电平触发，内部下拉
    
        zero_up       = 0x88u,     //低电平触发，内部上拉
        rising_up     = 0x89u,     //上升沿触发，内部上拉
        falling_up    = 0x8Au,     //下降沿触发，内部上拉
        either_up     = 0x8Bu,     //跳变沿触发，内部上拉
        one_up        = 0x8Cu      //高电平触发，内部上拉
    } exti_cfg;
   
  
    void gpio_init (GPIO_MemMapPtr port, int index, int dir,int data);
    
    void gpio_ctrl (GPIO_MemMapPtr port, int index, int data);
    
    void gpio_reverse (GPIO_MemMapPtr port, int index);

    void exti_init(GPIO_MemMapPtr port, int index, exti_cfg cfg, int priority);
    
#endif 
