#include "includes.h"
#include <math.h>

#define HMC5883L_SlaveAddress  0x3C
#define	BMP085_SlaveAddress    0xEE
#define	MPU6050_SlaveAddress   0xD0

#define BMP085_OSS 2

extern int16 gyroCAL[3],accCAL[3];

extern float angle[3],angleREF[3];

uint8 mpu6050_init(void)
{
  
    if(i2c_WriteByte(MPU6050_SlaveAddress, 0x6B, 0x80))             //PWR_MGMT_1    -- DEVICE_RESET 1
       return 1;
    
    for(uint32 i=0;i<=0xffff;i++);
    
    if(i2c_WriteByte(MPU6050_SlaveAddress, 0x6B, 0x03))             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
        return 1;
    if(i2c_WriteByte(MPU6050_SlaveAddress, 0x1A, 0x00))             //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
        return 1;
    if(i2c_WriteByte(MPU6050_SlaveAddress, 0x1B, 0x18))             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
        return 1;  
     
    if(i2c_WriteByte(MPU6050_SlaveAddress, 0x1C, 0x10))             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0  
        return 1; 
    
    return 0;
}


uint8 mpu6050_read(int16 *gyroData,int16 *accData)
{
    unsigned char temp[6];
    
    if(i2c_ReadMultiByte(MPU6050_SlaveAddress,0x43,temp,6))
        return 1;
    
    *gyroData=(temp[0]<<8|temp[1]);
    *(gyroData+1)=-(temp[2]<<8|temp[3]);
    *(gyroData+2)=(temp[4]<<8|temp[5]);
    
    if(i2c_ReadMultiByte(MPU6050_SlaveAddress,0x3B,temp,6))
        return 1;
          
    *accData=(temp[2]<<8 | temp[3]);
    *(accData+1)=(temp[0]<<8 | temp[1]);
    *(accData+2)=(temp[4]<<8 | temp[5]);
    
    // ACC_Common();
    return 0;
}

uint8 hmc5883l_init(void)
{
  if(i2c_WriteByte(HMC5883L_SlaveAddress,0x02,0x00))   //Continous-Conversion Mode on
    return 1;
  
  pause();
    
  return 0;
}


uint8 hmc5883l_read(int16 *Data)
{
    unsigned char temp[6];
    
    if(i2c_ReadMultiByte(HMC5883L_SlaveAddress,0x03,temp,6))
        return 1;

    *Data=(temp[4]<<8|temp[5]);     //Combine MSB and LSB of X Data output register
    *(Data+1)=(temp[0]<<8|temp[1]); //
    *(Data+2)=(temp[2]<<8|temp[3]); //
    
    
    /* *Data=temp[4]<<8|temp[5];     //Combine MSB and LSB of X Data output register
    *(Data+1)=temp[0]<<8|temp[1]; //
    *(Data+2)=temp[2]<<8|temp[3]; //
    */
    return 0;
}


extern int16 tempADC;
extern int32 altADC;

int16 ac1,ac2,ac3;
uint16 ac4,ac5,ac6;
int16 b1,b2,mb,mc,md;
int32 pressure,temperature;

uint8 ReadTemp[2],ReadPressure[3];
int32 ut,up;    //定义长整型变量
int32 x1, x2, b5, b6, x3, b3, p;
uint32 b4, b7;           //定义无符号长整型变量
 

uint8 bmp085_init(void)
{
 
    uint8 temp[2]; 
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xaa,temp,2))
        return 1;
    ac1 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xac,temp,2))
        return 1;
    ac2 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xae,temp,2))
          return 1;
    ac3 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xb0,temp,2))
          return 1;
    ac4 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xb2,temp,2))
          return 1;
    ac5 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xb4,temp,2))
          return 1;
    ac6 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xb6,temp,2))
          return 1;
    b1 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xb8,temp,2))
          return 1;
    b2 = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xba,temp,2))
          return 1;
    mb = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xbc,temp,2))
          return 1;
    mc = (temp[0]<<8)|temp[1];
    
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xbe,temp,2))
        return 1;
    md = (temp[0]<<8)|temp[1];        
    //连续读取EEPROM中的校准数据，并存放到相应的变量中，以供后面补偿使用

    return 0;
}

uint8 bmp085_startTemp(void)
{
  if(i2c_WriteByte(BMP085_SlaveAddress,0xf4,0x2e))                          //向地址0xf4写0x2e，进行温度转换
      return 1;
  
  return 0;
}


uint8 bmp085_startPres(void)
{
    if(i2c_WriteByte(BMP085_SlaveAddress,0xf4,0x34+(BMP085_OSS<<6)))   //向地址0xf4写0x34，进行第一次气压转换
         return 1;
    return 0;
}

uint8 bmp085_readTemp(void)
{
    
    if(!(GPIO_PDIR_REG(PORTE)&(1<<5)))
        return 1;

    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xf6,ReadTemp,2))             //从地址0xf6开始读出温度数据并存到数组ReadTemp中，共2个字节
        return 1;
    
    ut=ReadTemp[0]<<8|ReadTemp[1];  //合成温度数据
    x1=((long)ut-ac6)*ac5>>15;      //以下根据EEPROM中的值对获取的温度数据的进行补偿换算
    x2=((long)mc<<11)/(x1+md);
    b5=x1+x2;
    tempADC=(b5+8)>>4;  
    
    return 0;
}
  
uint8 bmp085_readPres(void)
{
    if(!(GPIO_PDIR_REG(PORTE)&(1<<5)))
        return 1;
  
    if(i2c_ReadMultiByte(BMP085_SlaveAddress,0xf6,ReadPressure,3))         //从地址0xf6开始读出气压数据并存到数组ReadPressure中，共3个字节
        return 1;
    
    up=(ReadPressure[0]<<16|ReadPressure[1]<<8|ReadPressure[2])>>(8-BMP085_OSS);  //合成气压数据
    up&=0x0007FFFF;
    
    x1=((long)ut-ac6)*ac5>>15;      //以下根据EEPROM中的值对获取的温度数据的进行补偿换算
    x2=((long)mc<<11)/(x1+md);
    b5=x1+x2;
    
    b6=b5-4000;       //以下根据EEPROM中的值对获取的气压数据的进行补偿换算
    x1=(b2*(b6*b6>>12))>>11;
    x2=ac2*b6>>11;
    x3=x1+x2;
    b3=((((long)ac1*4+x3)<<BMP085_OSS)+2)/4;
    x1=ac3*b6>>13;
    x2=(b1*(b6*b6>>12))>>16;
    x3=((x1+x2)+2)>>2;
    b4=(ac4*(unsigned long)(x3+32768))>>15;
    b7=((unsigned long)up-b3)*(50000>>BMP085_OSS);
    if(b7<0x80000000)
        p=(b7*2)/b4;
    else  
        p=(b7/b4)*2;
    x1=(p>>8)*(p>>8);
    x1=(x1*3038)>>16;
    x2=(-7357*p)>>16;
    pressure=p+((x1+x2+3791)>>4);
    
    altADC = (int32) ((1.0f - pow(pressure/101325.0f, 0.190295f)) * 4433000.0f * 10.0);
    
    return 0;
}

uint8 bmp085_update(void)
{
    static uint8 bmp085_flag=2;
    static uint8 error_count=0; //count for bmp085 failure
      
    switch(bmp085_flag)
    {
        case 1:
        {
            bmp085_startTemp();
            bmp085_flag=2;
            break;
        }
        case 2:
        { 
            if(!bmp085_readTemp())
            {
                bmp085_flag=3;
                error_count=0;
            }
            else
            {
                error_count++;
                if(error_count>=20)
                    bmp085_flag=1; 
            }
            break;
        }
        case 3:
        {
            bmp085_startPres();
            bmp085_flag=4;
            break;
        }
        case 4:
        {
            if(!bmp085_readPres())
            {
                bmp085_flag=1;
                error_count=0;
            }
            else
            {
                error_count++;
                if(error_count>=20)
                    bmp085_flag=1; 
            }
            break;
        }
        default: break;
    }
    
    if(error_count>=40)
      return 1;
    else
      return 0;
}

uint8 mpu6050_calibration(int16 *gyroCAL,int16 *accCAL)
{
    uint32 u=0,i=0;
    int16 gyroTMP[3]={0,0,0};
    int16 accTMP[3]={0,0,0};
    int32 gyroSUM[3]={0,0,0};
    int32 accSUM[3]={0,0,0};
    
    light_ctrl(WARNLIGHT,LIGHT_ON);    
    
    for(u=0;u<0x1ff;u++)
    {
        if(mpu6050_read(gyroTMP,accTMP))
            return 1;
        
        for(i=0;i<3;i++)
        {
          gyroSUM[i]+=gyroTMP[i];
          accSUM[i]+=accTMP[i];
        }
        
        for(i=0;i<0xffff;i++);
    }
    
    for(i=0;i<3;i++)
    {
        *(gyroCAL+i)=gyroSUM[i]/0x1ff;
        *(accCAL+i)=accSUM[i]/0x1ff;
    }
    
    angleREF[YAW]=angle[YAW];
    
    return 0;
}

uint8 sensors_init(void)
{ 
    uint32 u;
    
    gpio_init(PORTD,3,1,0); //disable IMU power
    gpio_init(PORTE,4,0,0);//connected to MPU6050 INT pin
    gpio_init(PORTE,5,0,0);//connected to BMP085 EOC pin
    gpio_init(PORTE,6,0,0);//connected to HM5883L DRY pin
       
    
    for(u=0;u<=0x2fffff;u++);
    
    gpio_ctrl(PORTD,3,1);//enable IMU power
    
    for(u=0;u<=0x2fffff;u++);
    
    i2c_init();
    
    if(hmc5883l_init())
        return 1;
    if(mpu6050_init())
        return 1;
    if(bmp085_init())
        return 1;
    
    for(u=0;u<=0x2fffff;u++);
    
    mpu6050_calibration(gyroCAL,accCAL);
    
    return 0;
}
