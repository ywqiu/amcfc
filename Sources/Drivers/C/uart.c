#include "uart.h"


void uart_init (UART_MemMapPtr uartch, uint32 sysclk, uint32 baud)
{
    register uint16 sbr, brfa;
    uint8 temp;

    //enable ports
    if (uartch == UART0_BASE_PTR)
    {
        PORTD_PCR6 = PORT_PCR_MUX(0x3);
        PORTD_PCR7 = PORT_PCR_MUX(0x3); 
    }
    else if(uartch == UART1_BASE_PTR)
    {
        PORTC_PCR4 = PORT_PCR_MUX(0x3); 
        PORTC_PCR3 = PORT_PCR_MUX(0x3); 
    }
    else if(uartch == UART2_BASE_PTR)
    {
        PORTD_PCR3 = PORT_PCR_MUX(0x3); 
        PORTD_PCR2 = PORT_PCR_MUX(0x3); 
    }
    else if(uartch == UART3_BASE_PTR)
    {		
        PORTC_PCR17 = PORT_PCR_MUX(0x3); 
        PORTC_PCR16 = PORT_PCR_MUX(0x3); 
    }
    else if(uartch == UART4_BASE_PTR)
    {
        PORTE_PCR24 = PORT_PCR_MUX(0x3); 
        PORTE_PCR25 = PORT_PCR_MUX(0x3); 
    }
    else if(uartch == UART5_BASE_PTR)
    {
        PORTE_PCR8 = PORT_PCR_MUX(0x3); 
        PORTE_PCR9 = PORT_PCR_MUX(0x3); 
    }
	 
    //enable UART clock
    if(uartch == UART0_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    else if (uartch == UART1_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
    else if (uartch == UART2_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_UART2_MASK;
    else if(uartch == UART3_BASE_PTR)
        SIM_SCGC4 |= SIM_SCGC4_UART3_MASK;
    else if(uartch == UART4_BASE_PTR)
        SIM_SCGC1 |= SIM_SCGC1_UART4_MASK;
    else
        SIM_SCGC1 |= SIM_SCGC1_UART5_MASK;
								
    //disable transmit and receive
    UART_C2_REG(uartch) &= ~(UART_C2_TE_MASK
				| UART_C2_RE_MASK );
	
    //���ó�8λ��У��ģʽ
    UART_C1_REG(uartch) = 0;
	
    //���㲨���ʣ�����0��1ʹ���ں�ʱ�ӣ���������ʹ������ʱ�ӣ�ϵͳʱ��Ϊ
    //����ʱ�ӵ�2��
    if ((uartch == UART0_BASE_PTR) | (uartch == UART1_BASE_PTR))//
        sysclk+=sysclk;
	
    sbr = (uint16)((sysclk*1000)/(baud * 16));
    temp = UART_BDH_REG(uartch) & ~(UART_BDH_SBR(0x1F));
    UART_BDH_REG(uartch) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    UART_BDL_REG(uartch) = (uint8)(sbr & UART_BDL_SBR_MASK);
    brfa = (((sysclk*32000)/(baud * 16)) - (sbr * 32));
    temp = UART_C4_REG(uartch) & ~(UART_C4_BRFA(0x1F));
    UART_C4_REG(uartch) = temp |  UART_C4_BRFA(brfa);    
	
    //enable receive FIFO��depth=16
    if(uartch==UART0_BASE_PTR)
    {
        UART0_PFIFO=0x0B;
    }
        
    //enable transmit and receive
    UART_C2_REG(uartch) |= (UART_C2_TE_MASK | UART_C2_RE_MASK );
}


uint8 uart_re1 (UART_MemMapPtr uartch,uint8 *ch)
{
    uint32 k;
    
    for (k = 0; k < 0xfbbb; k++)//��ʱ������
    if((UART_S1_REG(uartch) & UART_S1_RDRF_MASK)!= 0)//�жϽ��ջ������Ƿ���
    {
        *ch = UART_D_REG(uartch);
        return 1; 			//���ܳɹ�
    } 
    if(k>=0xfbbb) 
    {
        return 0;			//����ʧ��
    } 
    return 0;
}


void uart_send1 (UART_MemMapPtr uartch, uint8 ch)
{
    //�ȴ����ͻ�������
    while(!(UART_S1_REG(uartch) & UART_S1_TDRE_MASK));
    //��������
    UART_D_REG(uartch) = (uint8)ch;
 }


uint8 uart_reN (UART_MemMapPtr uartch ,uint8* buff,uint16 len)
{
    uint16 m=0; 
    while (m < len)
    { 	          
        if(0==uart_re1(uartch,&buff[m]))
  	    return 0;  //����ʧ��
  	else
            m++;
    } 
    
    return 1;          //���ճɹ�   
}


void uart_sendN (UART_MemMapPtr uartch ,uint8* buff,uint16 len)
{
    int i;
    
    for(i=0;i<len;i++)
    {
        uart_send1(uartch,buff[i]);
    }
}

void enableuartreint(UART_MemMapPtr uartch,uint8 irqno)
{
    UART_C2_REG(uartch)|=UART_C2_RIE_MASK;   //����UART�����ж�
    enable_irq(irqno);			 //���������ŵ�IRQ�ж�
}


void disableuartreint(UART_MemMapPtr uartch,uint8 irqno)
{
    UART_C2_REG(uartch)&=~UART_C2_RIE_MASK;   //��ֹUART�����ж�
    disable_irq(irqno);			  //�ؽ������ŵ�IRQ�ж�
}


void uart_sendstring (UART_MemMapPtr uartch ,uint8* buff)
{
    int i;

    for(i=0;buff[i] != 0;i++)
    {
        uart_send1(uartch,buff[i]);
    }
}

//transform number to ASCII code into array
uint8 get_byte_array_from_number(uint8 array [], int * length, int number)
{
    int i = 0;
    int j = 0;
    uint8 pBuff [10];
    if(number < 0)
    {
        return 1;
    }
    i = 0;
    if(number == 0) //���������0
    {
        i = 1;
        *length = i;
        array[0] = '0';
        return 0;
    }
    
    while(number > 0)
    {
        pBuff[i++] = number % 10;
        number = number/10; 
    }
    
    *length = i;
    //��������ASCII�����ݴ����array��
    for(j = 0; j < i; j++)
    {
        array[j] = pBuff[i - j - 1] + '0';
    }
    //array[j] = '\0';
    return 0;
}

//transform a number like 123 into a stirng like "123" and transmit  
uint8 uart_sendnumber(UART_MemMapPtr uartch , int number)
{
    uint8 array[10];
    int len = 0;
    if(get_byte_array_from_number(array, &len, number) == 0)
    {
    	uart_sendN(uartch, (uint8*)array, len);
    }
    else
    {
        return 0;
    } 
    return 1;
}










