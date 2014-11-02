#include "i2c.h"


void i2c_init(void)
{
    SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;     //��I2C0ģ��ʱ��
    PORTB_PCR0 = PORT_PCR_MUX(2);//I2C0 SCLʹ��PTB0
    PORTB_PCR1 = PORT_PCR_MUX(2);    //I2C0 SDAʹ��PTB1
    I2C0_F  = 0x17; //MUL=1 , ICR=128 , I2C0_Baud=390.6kHz  
    I2C0_C1 = I2C_C1_IICEN_MASK;//ʹ��I2C0    
    pause();
}

void send_signal(uint8 Signal)
{
    if (Signal == 'S')
    {    
        i2c0_Start(); //����ģʽѡ��λMST��0��Ϊ1,���Բ�����ʼ�ź�  
    }  
    else if (Signal == 'O')
    {
        i2c0_Stop();//����ģʽѡ��λMST��1��Ϊ0,���Բ���ֹͣ�ź�
    }
}

uint8 i2c_wait(uint8 x)
{
    uint16 ErrTime, i;
    ErrTime = 255 * 2;            //�����ѯ��ʱʱ��
  
    for (i = 0;i < ErrTime;i++)
    {
        if (x == 'A')           //�ȴ�Ӧ���ź�
        {
            if(( I2C0_S & I2C_S_RXAK_MASK)==0)
                return 0;      //������һ���ֽں�,�յ��˴ӻ���Ӧ���ź�               
        }
        else if (x == 'T')      //�ȴ��������һ���ֽ��ź�
        {
            if ((I2C0_S & I2C_S_IICIF_MASK) != 0)    
            {
                (I2C0_S |=(0 | I2C_S_IICIF_MASK));  //��IICIF��־λ
                return 0;       //�ɹ�������һ���ֽ�
            }       
        }
    }
    return 1;               //��ʱ,û���յ�Ӧ���źŻ�����һ���ֽ�   	
}



uint8 i2c_WriteByte(uint8 SlaveAddress,uint8 AccessAddress,uint8 Data)
{
    send_signal('S');
    I2C0_D  = SlaveAddress ;   //�����豸��ַ,��֪ͨ�ӻ���������
    if (i2c_wait('T'))                    
        return 1;                          
    if (i2c_wait('A'))                 
        return 1;    
    I2C0_D  = AccessAddress;        //���ͷ��ʵ�ַ    
    if (i2c_wait('T'))                
        return 1;                        
    if (i2c_wait('A'))           
        return 1;  
    I2C0_D  = Data;          
    if (i2c_wait('T'))           
        return 1;                       
    if (i2c_wait('A'))             
        return 1;  
    send_signal('O');           //����ֹͣ�ź�
    
    pause();
    
    return 0;
}

uint8 i2c_ReadByte(uint8 SlaveAddress,uint8 AccessAddress,uint8 *Data)
{
    I2C0_C1     |= 0x10;           //TX = 1,MCU����Ϊ����ģʽ
    send_signal('S');               //���Ϳ�ʼ�ź� 
    I2C0_D  = SlaveAddress & 0xfe;   //�����豸��ַ,��֪ͨ�ӻ���������
           
    if(i2c_wait('T'))                  //�ȴ�һ���ֽ����ݴ������  
    {      
        return 1;                         //û�д��ͳɹ�,��һ���ֽ�ʧ��   
    }
    if (i2c_wait('A'))                 //�ȴ��ӻ�Ӧ���ź� 
    {
        return 1;                         //û�еȵ�Ӧ���ź�,��һ���ֽ�ʧ�� 
    }
    I2C0_D  = AccessAddress;        //���ͷ��ʵ�ַ    
    if (i2c_wait('T'))                //�ȴ�һ���ֽ����ݴ������ 
    {
        return 1;                        //û�д��ͳɹ�,��һ���ֽ�ʧ��
    }
    if (i2c_wait('A'))                //�ȴ��ӻ�Ӧ���ź�   
    { 
        return 1;                        //û�еȵ�Ӧ���ź�,��һ���ֽ�ʧ��  
    }
    
    pause();
    I2C0_C1 |= 0x04;//��MCU������ģ ʽ�£����λд1������һ�����¿�ʼ�ź�  
    I2C0_D = SlaveAddress | 0x01; //֪ͨ�ӻ���Ϊ��������    
    if (i2c_wait('T'))               //�ȴ�һ���ֽ����ݴ������  
    {
        return 1;                       //û�д��ͳɹ�,��һ���ֽ�ʧ�� 
    }
    if (i2c_wait('A'))               //�ȴ��ӻ�Ӧ���ź�  
    {
        return 1;                      //û�еȵ�Ӧ���ź�,��һ���ֽ�ʧ��
    }
    I2C0_C1 &= 0xef;           //TX = 0,MCU����Ϊ����ģʽ    
    *Data = I2C0_D;            //����IIC1D,׼����������   
    if (i2c_wait('T'))              //�ȴ�һ���ֽ����ݴ������  
    {  
        return 1;                      //û�д��ͳɹ�,��һ���ֽ�ʧ��  
    }
    send_signal('O');           //����ֹͣ�ź�    
    *Data = I2C0_D;            //�������յ���һ������    
          
    pause();
    
    return 0;                          //��ȷ���յ�һ���ֽ����� 
}


uint8 i2c_ReadMultiByte(uint8 SlaveAddress,uint8 AccessAddress,uint8 *Data,uint8 Length)
{
    uint8 i;
    
    I2C0_C1     |= 0x10;
    send_signal('S');               //���Ϳ�ʼ�ź� 
  
    I2C0_D = SlaveAddress;   //�����豸��ַ,��֪ͨ�ӻ���������
    if (i2c_wait('T'))                  //�ȴ�һ���ֽ����ݴ������  
        return 1;                         //û�д��ͳɹ�,��һ���ֽ�ʧ��   
    if (i2c_wait('A'))                 //�ȴ��ӻ�Ӧ���ź� 
        return 1;                         //û�еȵ�Ӧ���ź�,��һ���ֽ�ʧ��
    I2C0_D = AccessAddress;        //���ͷ��ʵ�ַ    
    if (i2c_wait('T'))                //�ȴ�һ���ֽ����ݴ������ 
        return 1;                        //û�д��ͳɹ�,��һ���ֽ�ʧ��
    if (i2c_wait('A'))                //�ȴ��ӻ�Ӧ���ź�   
        return 1;  
      
    //pause();
    I2C0_C1 |= I2C_C1_RSTA_MASK;
      
    I2C0_D = SlaveAddress+1; //֪ͨ�ӻ���Ϊ��������    
    if (i2c_wait('T'))               //�ȴ�һ���ֽ����ݴ������  
        return 1;                       //û�д��ͳɹ�,��һ���ֽ�ʧ�� 
    if (i2c_wait('A'))               //�ȴ��ӻ�Ӧ���ź�  
        return 1;                      //û�еȵ�Ӧ���ź�,��һ���ֽ�ʧ��
      
    I2C0_C1 &= ~I2C_C1_TX_MASK;
    I2C0_C1 &= ~I2C_C1_TXAK_MASK;
  
    *Data = I2C0_D;
    if (i2c_wait('T'))               //�ȴ�һ���ֽ����ݴ������  
        return 1;                       //û�д��ͳɹ�,��һ���ֽ�ʧ�� 
    
    for(i=0;i<Length-2;i++)
    {
        *(Data+i)=I2C0_D;
        if (i2c_wait('T'))               //�ȴ�һ���ֽ����ݴ������  
              return 1;   
    }
                     //û�д��ͳɹ�,��һ���ֽ�ʧ�� 
    *(Data+Length-2) = I2C0_D;
    I2C0_C1 |= I2C_C1_TXAK_MASK;
    
    if (i2c_wait('T'))               //�ȴ�һ���ֽ����ݴ������  
        return 1;                       //û�д��ͳɹ�,��һ���ֽ�ʧ�� 
    i2c0_Stop();
    *(Data+Length-1) = I2C0_D;
    
    pause();
    
    return 0; 
}


void pause(void)
{
    int n;
    for(n=1;n<50;n++) 
    {
        asm(" nop");
    }
}

