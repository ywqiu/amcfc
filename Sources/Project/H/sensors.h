#ifndef SENSORS_H_
#define SENSORS_H_


	#include "common.h"   


               
        uint8 hmc5883l_init(void);
        uint8 hmc5883l_read(int16 *Data);
        
        uint8 mpu6050_init(void);
        uint8 mpu6050_read(int16 *gyroData,int16 *accData);

        uint8 bmp085_init(void);
        uint8 bmp085_startTemp(void);
        uint8 bmp085_startPres(void);
        uint8 bmp085_readTemp(void);
        uint8 bmp085_readPres(void);
        uint8 bmp085_update(void);
        
        uint8 mpu6050_calibration(int16 *gyroCAL,int16 *accCAL);
        
        uint8 sensors_init(void);
#endif