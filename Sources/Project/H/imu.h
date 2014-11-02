#ifndef __IMU_H__
#define __IMU_H__

  #define L3G_FIX_X -0.5000 
  #define L3G_FIX_Y -0.2369 
  #define L3G_FIX_Z 1.3269 
  
  #define L3G_P_X 1.11 //18
  #define L3G_P_Y 1.08 //5
  #define L3G_P_Z 1.15
  
  #define H_y 0.7467
  #define H_z -0.6652  

  void getEstimatedAttitude(void);
  void getEstimatedAltitude(void);
  
#endif 