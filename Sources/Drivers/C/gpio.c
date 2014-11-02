#include "gpio.h"


//GPIO initial, dir: 0-input, 1-output, data: 0-low, 1-high  
void gpio_init (GPIO_MemMapPtr port, int index, int dir,int data)
{
    PORT_MemMapPtr p;
    switch((uint32)port)
    {
      case 0x400FF000u:
          p = PORTA_BASE_PTR;
          break;
      case 0x400FF040u:
          p = PORTB_BASE_PTR;
          break;
      case 0x400FF080u:
          p = PORTC_BASE_PTR;
          break;
      case 0x400FF0C0u:
          p = PORTD_BASE_PTR;
          break;
      case 0x400FF100u:
          p = PORTE_BASE_PTR;
          break;
      default:
          break;
    }
  
    PORT_PCR_REG(p,index)=(0|PORT_PCR_MUX(1));

    if(dir == 1)//output
    {
        GPIO_PDDR_REG(port) |= (1<<index);
    	if(data == 1)//output
	    GPIO_PDOR_REG(port) |= (1<<index);
        else
	    GPIO_PDOR_REG(port) &= ~(1<<index);
    }
    else
        GPIO_PDDR_REG(port) &= ~(1<<index);
}


void gpio_ctrl(GPIO_MemMapPtr port, int index, int data)
{
    if(data == 1)//output
        GPIO_PDOR_REG(port) |= (1<<index);
    else
        GPIO_PDOR_REG(port) &= ~(1<<index);
}


void gpio_reverse (GPIO_MemMapPtr port, int index)
{
    GPIO_PDOR_REG(port) ^= (1<<index);
}


void exti_init(GPIO_MemMapPtr port, int index, exti_cfg cfg, int priority)
{
    PORT_MemMapPtr p;
  
    switch((uint32)port)
    {
        case 0x400FF000u:p = PORTA_BASE_PTR;break;
        case 0x400FF040u:p = PORTB_BASE_PTR;break;
        case 0x400FF080u:p = PORTC_BASE_PTR;break;
        case 0x400FF0C0u:p = PORTD_BASE_PTR;break;
        case 0x400FF100u:p = PORTE_BASE_PTR;break;
        default:break;
    }
  
    PORT_PCR_REG(p,index) = (0 | PORT_PCR_MUX(1));  //初始化端口
    PORT_PCR_REG(p,index) = PORT_PCR_MUX(1) | PORT_PCR_IRQC(cfg & 0x7f ) | PORT_PCR_PE_MASK | ((cfg & 0x80 ) >> 7); // 复用GPIO , 确定触发模式 ,开启上拉或下拉电阻
    GPIO_PDDR_REG(port) &= ~(1<<index); //输入模式
       
    switch((uint32)port)
    {
        case 0x400FF000u:enable_irq(87);set_irq_priority(87,priority);break;
        case 0x400FF040u:enable_irq(88);set_irq_priority(88,priority);break;
        case 0x400FF080u:enable_irq(89);set_irq_priority(89,priority);break;
        case 0x400FF0C0u:enable_irq(90);set_irq_priority(90,priority);break;
        case 0x400FF100u:enable_irq(91);set_irq_priority(91,priority);break;
        default:break;
    }
}