#include "includes.h"

extern int periph_clk_khz;

//float Kp_angle[3]={0,0,0};

//float Kp_angle[3]={5.6,5.6,5.0};
//float Ki_angle[3]={0.006,0.006,0.003};

float Kp_angle[3]={6.0,6.0,12.6};
float Ki_angle[3]={0.0002,0.0002,0.004};
float Kd_angle[3]={720.0,720.0,1480.0};
float Kdd_angle[3]={12000.0,12000.0,0.0};
float angleITEM[3]={0,0,0};


float Kp_speed[3]={400.0,400.0,400.0};
float Ki_speed[3]={0,0,0};
float Kd_speed[3]={40.0,40.0,40.0};
float speedITEM[3]={0,0,0};

uint8 flight_status=LOCK;
uint8 flight_mode=0;
uint8 light_OnOff=0;

//rc channels input port
const uint8 channel_port[9]={23,22,21,20,11,10,9,3,2};

//rc channels input data

int32 ppmADC[9];
int32 ppm_input[9];
int32 ppm_error=0;



uint8 failure_bmp085=0;


char string_send[100];

int32 send_flag=1;



int16 accADC[3],gyroADC[3],magADC[3];
int32 tempADC,altADC;


int16 accCAL[3]={0,0,0},gyroCAL[3]={0,0,0};

int16 accSmooth[3]={0,0,0};
int16 mgSmooth[3]={0,0,0};

float accEST[3];
float speedEST[3];
float speedREF[3];
float speedERR[3];

uint8 bmp085_flag=1;
  
float angle[3];      //姿态欧拉角
float angleReferance[3];
float anglePrevious[3];
float deltaAngle[3];

float angleREF[3];
float angleERR[3];
float angleI[3];
float angleDIF[3];
float angleDIFPRE[3];
float angleDIFDIF[3];
float anglePRE[3];
float angleERRPRE[3];

float altEST; //altitude
            
int32 m1,m2,m3,m4;
            


float signal;

void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec);


float test_signal(float amp);

void main(void)
{ 	
    int u=0,lightt=0;
    uint8 flight_mode_pre=0;   
    
    DisableInterrupts;       
    
    light_init();                       //告警指示灯初始化
    
    pwm_init();
    
    ppm_init();  //initialize rc ppm input;

    sensors_init();
 
    uart_init(UART0,periph_clk_khz,115200);
    
    
    pit_init(0,125000);                             //初始化pit0，周期0.025s
    enable_pit_interrupt(PIT0);                       //开pit中断
    
    EnableInterrupts;			              //开总中断
    
  
    uart_sendstring(UART0,"RESET!!!");

    //pwm_init();
    
  
       
    while(1)
    {
      
      char msg[100];
  
      
    
      if(send_flag)
      {
        mpu6050_read(gyroADC,accADC);
        hmc5883l_read(magADC);  
        
             
        failure_bmp085=bmp085_update();
        
        
        getEstimatedAttitude();
        getEstimatedAltitude();
        
        ppm_transform();
        
        signal=test_signal(0.3);
          
        if(flight_status==NORMAL)
        {
            angleREF[ROLL]=-(ppmADC[AILE]-1000.0)/1000.0*45.0;
            angleREF[PITCH]=(ppmADC[ELEV]-1000.0)/1000.0*45.0;
            angleREF[YAW]-=(ppmADC[RUDD]-1000.0)/1000.0/400.0*90.0;
            
            if(flight_mode==2)
                angleREF[ROLL]+=signal;
            
            if(angleREF[YAW]>180.0)
                angleREF[YAW] -= 360.0;
            if(angleREF[YAW]<=-180.0)
                angleREF[YAW] += 360.0;
            
            angleERR[ROLL]=angleREF[ROLL]-angle[ROLL];
            angleERR[PITCH]=angleREF[PITCH]-angle[PITCH];
            angleERR[YAW]=angleREF[YAW]-angle[YAW];
            
            if(angleERR[YAW]<-180.0)
                angleERR[YAW] += 360.0;
            if(angleERR[YAW]>=180.0)
                angleERR[YAW] -= 360.0;
            
            angleI[ROLL]+=angleERR[ROLL];
            angleI[PITCH]+=angleERR[PITCH];
            angleI[YAW]+=angleERR[YAW];
            
            for(u=0;u<3;u++)
            {
                if(angleI[u]>1800.0)
                    angleI[u]=1800.0;
                if(angleI[u]<-1800.0)
                    angleI[u]=-1800.0;
            }
                  
            //************************************************修改为真正的D而非增稳
            angleDIF[ROLL]  = angleDIF[ROLL]  *0.88 + (anglePRE[ROLL]  - angle[ROLL]  ) *0.12;
            angleDIF[PITCH] = angleDIF[PITCH] *0.88 + (anglePRE[PITCH] - angle[PITCH] ) *0.12;
            angleDIF[YAW]   = angleDIF[YAW]   *0.50 + (anglePRE[YAW]   - angle[YAW]   ) *0.50;
            
            for(u=0;u<3;u++)
            {
                angleDIFDIF[u]=angleDIFDIF[u]*0.65 + (angleDIFPRE[u] - angleDIF[u]) *0.35;
            }
            
            
            for(u=0;u<3;u++)
            {
                anglePRE[u]    = angle[u];
                angleERRPRE[u] = angleERR[u];
                angleDIFPRE[u] = angleDIF[u];
            }
            
            
            for(u=0;u<2;u++)
                angleITEM[u]=angleERR[u]*Kp_angle[u] + angleDIF[u]*Kd_angle[u] + angleI[u]*Ki_angle[u] - angleDIFDIF[u]*Kdd_angle[u];
            
            
            //航向锁定
           
            if(flight_mode==1)
            {
                if(!(flight_mode_pre==1))
                {
                    angleREF[YAW]=angle[YAW];
                    angleERR[YAW]=0.0;
                    angleI[YAW]=0.0;
                }
                angleITEM[YAW]=angleERR[YAW]*Kp_angle[YAW] + angleDIF[YAW]*Kd_angle[YAW] + angleI[YAW]*Ki_angle[YAW];
                
                if(angleITEM[YAW]>800)
                    angleITEM[YAW]=800;
                if(angleITEM[YAW]<-800)
                    angleITEM[YAW]=-800;
                
                m1 = 781.0 + 0.3905*(float)ppmADC[THRO] - angleITEM[ROLL] - angleITEM[PITCH] + angleITEM[YAW] ;
                m2 = 781.0 + 0.3905*(float)ppmADC[THRO] + angleITEM[ROLL] - angleITEM[PITCH] - angleITEM[YAW];
                m3 = 781.0 + 0.3905*(float)ppmADC[THRO] + angleITEM[ROLL] + angleITEM[PITCH] + angleITEM[YAW];
                m4 = 781.0 + 0.3905*(float)ppmADC[THRO] - angleITEM[ROLL] + angleITEM[PITCH] - angleITEM[YAW] ;
            }
            else if(flight_mode==0)
            {
               angleITEM[YAW]=angleDIF[YAW]*Kd_angle[YAW];
               
               m1 = 781.0 + 0.3905*(float)ppmADC[THRO] - angleITEM[ROLL] - angleITEM[PITCH] + angleITEM[YAW] - 0.5*(ppmADC[RUDD] - 1000);
               m2 = 781.0 + 0.3905*(float)ppmADC[THRO] + angleITEM[ROLL] - angleITEM[PITCH] - angleITEM[YAW] + 0.5*(ppmADC[RUDD] - 1000);
               m3 = 781.0 + 0.3905*(float)ppmADC[THRO] + angleITEM[ROLL] + angleITEM[PITCH] + angleITEM[YAW] - 0.5*(ppmADC[RUDD] - 1000);
               m4 = 781.0 + 0.3905*(float)ppmADC[THRO] - angleITEM[ROLL] + angleITEM[PITCH] - angleITEM[YAW] + 0.5*(ppmADC[RUDD] - 1000);
            }
            else if(flight_mode==2)
            { 
                speedREF[YAW] = (ppmADC[THRO] - 1000.0)/1000.0*6.0;
                
                speedERR[YAW] = speedREF[YAW] - speedEST[YAW];
                                
                speedITEM[YAW] = speedREF[YAW]*Kp_speed[YAW] + accEST[YAW]*Kd_speed[YAW];
                if(speedITEM[YAW]>1500)
                    speedITEM[YAW]=1500;
                if(speedITEM[YAW]<-1500)
                    speedITEM[YAW]=-1500;
                
                m1 = 781.0 + 0.3905*500.0 + speedITEM[YAW] - angleITEM[ROLL] - angleITEM[PITCH] + angleITEM[YAW] - 0.5*(ppmADC[RUDD] - 1000);
                m2 = 781.0 + 0.3905*500.0 + speedITEM[YAW] + angleITEM[ROLL] - angleITEM[PITCH] - angleITEM[YAW] + 0.5*(ppmADC[RUDD] - 1000);
                m3 = 781.0 + 0.3905*500.0 + speedITEM[YAW] + angleITEM[ROLL] + angleITEM[PITCH] + angleITEM[YAW] - 0.5*(ppmADC[RUDD] - 1000);
                m4 = 781.0 + 0.3905*500.0 + speedITEM[YAW] - angleITEM[ROLL] + angleITEM[PITCH] - angleITEM[YAW] + 0.5*(ppmADC[RUDD] - 1000);
            }
            flight_mode_pre = flight_mode;
            
            if(m1<859)  m1 = 859;
            if(m1>1562) m1 = 1562;
            if(m2<859)  m2 = 859;
            if(m2>1562) m2 = 1562;
            if(m3<859)  m3 = 859;
            if(m3>1562) m3 = 1562;
            if(m4<859)  m4 = 859;
            if(m4>1562) m4 =1562;

            if(ppmADC[THRO]<200)
            {
                m1 = 859;
                m2 = 859;
                m3 = 859;
                m4 = 859;
            }
                     
            FTM0_C0V = m1;
            FTM0_C1V = m2;
            FTM0_C2V = m3;
            FTM0_C3V = m4;
            
        }
        else
        {
            FTM0_C0V = 781.0;
            FTM0_C1V = 781.0;
            FTM0_C2V = 781.0 ;
            FTM0_C3V = 781.0 ;
        }
        
         
        
        //sprintf(msg,"%d\r\n",altADC);
        //uart_sendstring(UART0,msg);
        
        //FTM0_C0V=781.0+0.3905*(float)ppmADC[THRO] ;
         //   FTM0_C1V=781.0+0.3905*(float)ppmADC[THRO] ;
         //   FTM0_C2V=781.0+0.3905*(float)ppmADC[THRO] ;
          //  FTM0_C3V=781.0+0.3905*(float)ppmADC[THRO] ;
      
        
        static uint16 send=0;
        
        send++;
        
        if(send>=20)
        {  
           ppm_gesture();
           light_flash();
           
           
           if(light_OnOff)
           {
               gpio_ctrl(PORTB,19,1);
               lightt++;
               if(lightt>=3)
                  gpio_ctrl(PORTB,18,0);
               if(lightt>=6)
               {
                 lightt=0;
                  gpio_ctrl(PORTB,18,1);
               }

           }
           else
           {
             gpio_ctrl(PORTB,18,0);
             gpio_ctrl(PORTB,19,0);
           }
            
           send=0;
            UART1_ReportIMU((int16)(-angle[YAW]*10.0),(int16)(angle[PITCH]*10.0),(int16)(angle[ROLL]*10.0),(int16)(altADC*0.001),(int16)tempADC,0,400);
        
           //
           
           //sprintf(msg,"%d %d %d %d %d\r\n",ppm_input[0],ppm_input[1],ppm_input[2],ppm_input[3],ppm_input[FMOD]);
           
           //sprintf(msg,"%.2f %.2f %.2f %d\r\n",angleREF[YAW],angle[YAW],angleERR[YAW],flight_mode);
             
           //sprintf(msg,"%d %d %d %d %d %d %.2f %.2f %.2f\r\n",gyroADC[0],gyroADC[1],gyroADC[2],accADC[0],accADC[1],accADC[2],angle[0],angle[1],angle[2]);
             
           //sprintf(msg,"%.5f %.5f\r\n",signal,angle[ROLL]);
           
           //sprintf(msg,"%.2f %.3f \r\n", cos(PI / 180.0 * angle[ROLL])*cos(PI / 180.0 * angle[PITCH])*accADC[YAW] + cos(PI / 180.0 * angle[ROLL])*sin(PI / 180.0 * angle[PITCH])*accADC[PITCH] + sin(PI / 180.0 * angle[ROLL])*cos(PI / 180.0 * angle[PITCH])*accADC[ROLL] , (float)altADC/1000.0);
           
           //sprintf(msg,"%.5f %.5f %.2f\r\n",(float)altADC/1000.0,altEST,accEST[YAW]);
           //uart_sendstring(UART0,msg);
        }
        
        
        send_flag=0;
      }
    }
}

float test_signal(float amp)
{
    static int i=0;
    float f=0,sum=0;
    
    i++;
    if(i>=4000)
      i=1;
    
    for(f=0.5;f<=5.0;f+=0.2)
    {
        sum+=amp*cos(2*PI*f* i*0.0025 + PI / 180.0 * f*f*f*f*f*f*f);
    }
    
    return sum;
}


void UART1_ReportIMU(int16_t yaw,int16_t pitch,int16_t roll
,int16_t alt,int16_t tempr,int16_t press,int16_t IMUpersec)
{
 	unsigned int temp=0xaF+2;
	char ctemp;
	uart_send1(UART0,0xa5);
	uart_send1(UART0,0x5a);
	uart_send1(UART0,14+2);
	uart_send1(UART0,0xA1);

	if(yaw<0)yaw=32768-yaw;
	ctemp=yaw>>8;
	uart_send1(UART0,ctemp);
	temp+=ctemp;
	ctemp=yaw;
	uart_send1(UART0,ctemp);
	temp+=ctemp;

	if(pitch<0)pitch=32768-pitch;
	ctemp=pitch>>8;
	uart_send1(UART0,ctemp);
	temp+=ctemp;
	ctemp=pitch;
	uart_send1(UART0,ctemp);
	temp+=ctemp;

	if(roll<0)roll=32768-roll;
	ctemp=roll>>8;
	uart_send1(UART0,ctemp);
	temp+=ctemp;
	ctemp=roll;
	uart_send1(UART0,ctemp);
	temp+=ctemp;

	if(alt<0)alt=32768-alt;
	ctemp=alt>>8;
	uart_send1(UART0,ctemp);
	temp+=ctemp;
	ctemp=alt;
	uart_send1(UART0,ctemp);
	temp+=ctemp;

	if(tempr<0)tempr=32768-tempr;
	ctemp=tempr>>8;
	uart_send1(UART0,ctemp);
	temp+=ctemp;
	ctemp=tempr;
	uart_send1(UART0,ctemp);
	temp+=ctemp;

	if(press<0)press=32768-press;
	ctemp=press>>8;
	uart_send1(UART0,ctemp);
	temp+=ctemp;
	ctemp=press;
	uart_send1(UART0,ctemp);
	temp+=ctemp;

	uart_send1(UART0,temp%256);
	uart_send1(UART0,0xaa);
}