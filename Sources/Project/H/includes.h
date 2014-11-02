//2013.3.12 增加手势（锁定、解锁、加表校准）
//2013.3.13 PID姿态自稳

#ifndef INCLUDE_H_
#define INCLUDE_H_

	
	#include "common.h"            
        #include <stdio.h>
        #include <math.h>                

        #include  "gpio.h"                  
	#include  "uart.h"                 
        #include  "adc.h" 
        #include  "i2c.h"
        #include  "pit.h"

        #include  "light.h"
        #include  "sensors.h"
        #include  "imu.h"
        #include  "ppm.h"
   
      	
        #define MAG 1  

        #define TIME_STEP 0.0025

        //define rc chennals
        #define ELEV       0
        #define AILE       1
        #define THRO       2
        #define RUDD       3
        #define FMOD       4
        #define AUX1       5
        #define AUX2       6
        #define AUX3       7
        #define AUX4       8

        //define axis names
        #define ROLL       0
        #define PITCH      1
        #define YAW        2
        #define THROTTLE   3
       // #define AUX1       4
        //#define AUX2       5
        #define CAMPITCH   6
        #define CAMROLL    7
        
        #define RC_LIMIT 200.0  

        //define flight status
        #define LOCK       0
        #define NORMAL     1
        #define ERROR      2
        #define FAULT      3

       //define gestures
        #define GES_WAIT   0
        #define GES_LOCK   1 
        #define GES_UNLOCK 2
        #define GES_CALI   3
  

        
    
        
        #define PI 3.141592653589794


#endif
