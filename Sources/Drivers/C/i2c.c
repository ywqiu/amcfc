#include "i2c.h"


void i2c_init(void)
{
    SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;     //打开I2C0模块时钟
    PORTB_PCR0 = PORT_PCR_MUX(2);//I2C0 SCL使用PTB0
    PORTB_PCR1 = PORT_PCR_MUX(2);    //I2C0 SDA使用PTB1
    I2C0_F  = 0x17; //MUL=1 , ICR=128 , I2C0_Baud=390.6kHz  
    I2C0_C1 = I2C_C1_IICEN_MASK;//使能I2C0    
    pause();
}

void send_signal(uint8 Signal)
{
    if (Signal == 'S')
    {    
        i2c0_Start(); //主机模式选择位MST由0变为1,可以产生开始信号  
    }  
    else if (Signal == 'O')
    {
        i2c0_Stop();//主机模式选择位MST由1变为0,可以产生停止信号
    }
}

uint8 i2c_wait(uint8 x)
{
    uint16 ErrTime, i;
    ErrTime = 255 * 2;            //定义查询超时时限
  
    for (i = 0;i < ErrTime;i++)
    {
        if (x == 'A')           //等待应答信号
        {
            if(( I2C0_S & I2C_S_RXAK_MASK)==0)
                return 0;      //传送完一个字节后,收到了从机的应答信号               
        }
        else if (x == 'T')      //等待传送完成一个字节信号
        {
            if ((I2C0_S & I2C_S_IICIF_MASK) != 0)    
            {
                (I2C0_S |=(0 | I2C_S_IICIF_MASK));  //清IICIF标志位
                return 0;       //成功发送完一个字节
            }       
        }
    }
    return 1;               //超时,没有收到应答信号或发送完一个字节   	
}



uint8 i2c_WriteByte(uint8 SlaveAddress,uint8 AccessAddress,uint8 Data)
{
    send_signal('S');
    I2C0_D  = SlaveAddress ;   //发送设备地址,并通知从机接收数据
    if (i2c_wait('T'))                    
        return 1;                          
    if (i2c_wait('A'))                 
        return 1;    
    I2C0_D  = AccessAddress;        //发送访问地址    
    if (i2c_wait('T'))                
        return 1;                        
    if (i2c_wait('A'))           
        return 1;  
    I2C0_D  = Data;          
    if (i2c_wait('T'))           
        return 1;                       
    if (i2c_wait('A'))             
        return 1;  
    send_signal('O');           //发送停止信号
    
    pause();
    
    return 0;
}

uint8 i2c_ReadByte(uint8 SlaveAddress,uint8 AccessAddress,uint8 *Data)
{
    I2C0_C1     |= 0x10;           //TX = 1,MCU设置为发送模式
    send_signal('S');               //发送开始信号 
    I2C0_D  = SlaveAddress & 0xfe;   //发送设备地址,并通知从机接收数据
           
    if(i2c_wait('T'))                  //等待一个字节数据传送完成  
    {      
        return 1;                         //没有传送成功,读一个字节失败   
    }
    if (i2c_wait('A'))                 //等待从机应答信号 
    {
        return 1;                         //没有等到应答信号,读一个字节失败 
    }
    I2C0_D  = AccessAddress;        //发送访问地址    
    if (i2c_wait('T'))                //等待一个字节数据传送完成 
    {
        return 1;                        //没有传送成功,读一个字节失败
    }
    if (i2c_wait('A'))                //等待从机应答信号   
    { 
        return 1;                        //没有等到应答信号,读一个字节失败  
    }
    
    pause();
    I2C0_C1 |= 0x04;//当MCU在主机模 式下，向该位写1将产生一个重新开始信号  
    I2C0_D = SlaveAddress | 0x01; //通知从机改为发送数据    
    if (i2c_wait('T'))               //等待一个字节数据传送完成  
    {
        return 1;                       //没有传送成功,读一个字节失败 
    }
    if (i2c_wait('A'))               //等待从机应答信号  
    {
        return 1;                      //没有等到应答信号,读一个字节失败
    }
    I2C0_C1 &= 0xef;           //TX = 0,MCU设置为接收模式    
    *Data = I2C0_D;            //读出IIC1D,准备接收数据   
    if (i2c_wait('T'))              //等待一个字节数据传送完成  
    {  
        return 1;                      //没有传送成功,读一个字节失败  
    }
    send_signal('O');           //发送停止信号    
    *Data = I2C0_D;            //读出接收到的一个数据    
          
    pause();
    
    return 0;                          //正确接收到一个字节数据 
}


uint8 i2c_ReadMultiByte(uint8 SlaveAddress,uint8 AccessAddress,uint8 *Data,uint8 Length)
{
    uint8 i;
    
    I2C0_C1     |= 0x10;
    send_signal('S');               //发送开始信号 
  
    I2C0_D = SlaveAddress;   //发送设备地址,并通知从机接收数据
    if (i2c_wait('T'))                  //等待一个字节数据传送完成  
        return 1;                         //没有传送成功,读一个字节失败   
    if (i2c_wait('A'))                 //等待从机应答信号 
        return 1;                         //没有等到应答信号,读一个字节失败
    I2C0_D = AccessAddress;        //发送访问地址    
    if (i2c_wait('T'))                //等待一个字节数据传送完成 
        return 1;                        //没有传送成功,读一个字节失败
    if (i2c_wait('A'))                //等待从机应答信号   
        return 1;  
      
    //pause();
    I2C0_C1 |= I2C_C1_RSTA_MASK;
      
    I2C0_D = SlaveAddress+1; //通知从机改为发送数据    
    if (i2c_wait('T'))               //等待一个字节数据传送完成  
        return 1;                       //没有传送成功,读一个字节失败 
    if (i2c_wait('A'))               //等待从机应答信号  
        return 1;                      //没有等到应答信号,读一个字节失败
      
    I2C0_C1 &= ~I2C_C1_TX_MASK;
    I2C0_C1 &= ~I2C_C1_TXAK_MASK;
  
    *Data = I2C0_D;
    if (i2c_wait('T'))               //等待一个字节数据传送完成  
        return 1;                       //没有传送成功,读一个字节失败 
    
    for(i=0;i<Length-2;i++)
    {
        *(Data+i)=I2C0_D;
        if (i2c_wait('T'))               //等待一个字节数据传送完成  
              return 1;   
    }
                     //没有传送成功,读一个字节失败 
    *(Data+Length-2) = I2C0_D;
    I2C0_C1 |= I2C_C1_TXAK_MASK;
    
    if (i2c_wait('T'))               //等待一个字节数据传送完成  
        return 1;                       //没有传送成功,读一个字节失败 
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

