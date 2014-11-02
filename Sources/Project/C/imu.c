#include "includes.h"



#define ACC_LPF_FACTOR 50
#define GYRO_LPF_FACTOR 5
#define MG_LPF_FACTOR 10

#define GYR_CMPF_FACTOR 1420.0f  //Set the Gyro Weight for Gyro/Acc complementary filter
#define GYR_CMPFM_FACTOR 1020.0f  //Set the Gyro Weight for Gyro/Magnetometer complementary filte

#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE (1.127 * 0.060976 *  PI / 180.0) //in rad when measurment range=2000dps

#define ACC_1G 4096.0
#define ACC_25DEG 4096.0 * 0.423
 
typedef struct fp_vector{
  float X;
  float Y;
  float Z;
} fp_vector;
    
typedef union {
  float A[3]; 
  fp_vector V;
} t_fp_vector;



extern int16 accADC[3],gyroADC[3],magADC[3];
extern int32 tempADC,altADC;
extern int16 accCAL[3],gyroCAL[3];
extern int16 accSmooth[3];
extern int16 mgSmooth[6];
  
extern float angle[3];      //×ËÌ¬Å·À­½Ç
extern float anglePrevious[3];
extern float deltaAngle[3];

extern float accEST[3];
extern float speedEST[3];

extern float altEST; //altitude


void rotateV(struct fp_vector *v,float* delta) {
  fp_vector v_tmp = *v;
  v->Z -= delta[ROLL]  * v_tmp.X + delta[PITCH] * v_tmp.Y;
  v->X += delta[ROLL]  * v_tmp.Z - delta[YAW]   * v_tmp.Y;
  v->Y += delta[PITCH] * v_tmp.Z + delta[YAW]   * v_tmp.X; 
}


void getEstimatedAttitude()
{
  uint8 axis;
  uint8 smallAngle25;
  static t_fp_vector EstG={0,0,1},EstM={1,0,0};
  float accMag=0;
  float deltaGyroAngle[3];
  
  
  //fix
  accADC[0]=(accADC[0] - accCAL[0]);
  accADC[1]=(accADC[1] - accCAL[1]);
  accADC[2]=(accADC[2]);
  
  //gyro
  gyroADC[0]=gyroADC[0]-gyroCAL[0];
  gyroADC[1]=gyroADC[1]-gyroCAL[1];
  gyroADC[2]=gyroADC[2]-gyroCAL[2];
  
  
  for(axis=0;axis<3;axis++)
  {
    deltaGyroAngle[axis] = gyroADC[axis] * GYRO_SCALE * TIME_STEP;
  } 
  
  for (axis = 0; axis < 3; axis++) 
  {
    accSmooth[axis] = (accSmooth[axis] * (ACC_LPF_FACTOR - 1) + accADC[axis]) / ACC_LPF_FACTOR;// LPF for ACC values
    accMag += ( (float) accSmooth[axis] *10 / ACC_1G ) * ( (float) accSmooth[axis] *10 / ACC_1G); // get g vector mod (10*p.u.)^2
    mgSmooth[axis] = (mgSmooth[axis] * (MG_LPF_FACTOR - 1) + magADC[axis]) / MG_LPF_FACTOR; // LPF for Magnetometer values
  }
  
  rotateV(&EstG.V,deltaGyroAngle);  
  #if MAG
    rotateV(&EstM.V,deltaGyroAngle);
  #endif
   
  if ( abs(accSmooth[ROLL]) < ACC_25DEG && abs(accSmooth[PITCH]) < ACC_25DEG && accSmooth[YAW]>0) 
  {
    smallAngle25 = 1;
    EstG.V.Z = ACC_1G;
  } 
  else
  {
    smallAngle25 = 0;
  }
  
  
  
  if ( ( 64 < accMag && accMag < 144 ) )//|| smallAngle25
  {
    for (axis = 0; axis < 3; axis++)
      EstG.A[axis] = (EstG.A[axis] * GYR_CMPF_FACTOR + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
  }
  
  #if MAG 
    for (axis = 0; axis < 3; axis++)
      EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR  + mgSmooth[axis]) * INV_GYR_CMPFM_FACTOR;
  #endif
  
  
  angle[ROLL]  =  atan2(EstG.V.X , EstG.V.Z) * 180.0 / PI;
  angle[PITCH] =  atan2(EstG.V.Y , EstG.V.Z) * 180.0 / PI;
  #if MAG 
    angle[YAW] = atan2( EstG.V.X * EstM.V.Z - EstG.V.Z * EstM.V.X , EstG.V.Z * EstM.V.Y - EstG.V.Y * EstM.V.Z )  * 180.0 / PI;
  #endif
}


void getEstimatedAltitude(void)
{
    static float dt=0.0025;
  
    static float altBARO;  
    
    static float X[3] = {0,0,0};
    static float P[3][3] = {{ 1.0, 0.0, 0.0 },
                            { 0.0, 1.0, 0.0 },
                            { 0.0, 0.0, 1.0 }};
    static float Q[3] = {0.001,0.001,0.001};
    static float R[3] = {7200.0,0.001,0.001};
    static float S[3] = {0.0,0.0,0.0};
    static float K[3] = {0.0,0.0,0.0};
    static float Y = 0.0;
    
    accEST[YAW] = (cos(PI / 180.0 * angle[ROLL])*cos(PI / 180.0 * angle[PITCH])*accSmooth[YAW] + cos(PI / 180.0 * angle[ROLL])*sin(PI / 180.0 * angle[PITCH])*accSmooth[PITCH] + sin(PI / 180.0 * angle[ROLL])*cos(PI / 180.0 * angle[PITCH])*accSmooth[ROLL]) /4096.0*9.8 -9.8;
    altBARO = altBARO*0.92 + (altADC/1000.0)*0.08;
    
    X[1] += accEST[YAW] * dt;
    
    X[0] += X[1] * dt;
    X[1] += X[2] * dt;
    
    P[0][0] += (P[0][1] + P[1][0])*dt + Q[0];
    P[0][1] += (P[0][2] + P[1][1])*dt;
    P[0][2] += P[1][2]*dt;
    P[1][0] += (P[1][1] + P[2][0])*dt;
    P[1][1] += (P[1][2] + P[2][1])*dt + Q[1];
    P[1][2] += P[2][2]*dt;
    P[2][0] += P[2][1]*dt;
    P[2][1] += P[2][2]*dt;
    P[2][2] += Q[2];
    
    Y = altBARO - X[0];
    
    S[0]=P[0][0] + R[0];
    S[1]=R[1];
    S[2]=R[2];
    
    K[0] = P[0][0]/S[0];
    K[1] = P[1][0]/S[0];
    K[2] = P[2][0]/S[0];
    
    X[0] += K[0]*Y; 
    X[1] += K[1]*Y; 
    X[2] += K[2]*Y; 
    
    P[0][0] -= P[0][0] * K[0];
    P[0][1] -= P[0][1] * K[0];
    P[0][2] -= P[0][2] * K[0];
    P[1][0] -= P[0][0] * K[1];
    P[1][1] -= P[0][1] * K[1];
    P[1][2] -= P[0][2] * K[1];
    P[2][0] -= P[0][0] * K[2];
    P[2][1] -= P[0][1] * K[2];
    P[2][2] -= P[0][2] * K[2];

    altEST  = X[0];
    speedEST[YAW] = X[1];
    accEST[YAW] += X[2];
}