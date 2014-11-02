#include "adc.h"


#define A                 0x0
#define B                 0x1

tADC_Config Master_Adc_Config;  

//============================================================================
//函数名称：hw_adc_init
//函数返回：0 成功 ，1 失败
//参数说明：MoudelNumber：模块号
//功能概要：AD初始化
//============================================================================
uint8 hw_adc_init(int MoudelNumber)
{
    if(MoudelNumber == 0)//模块0
    {
        SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK );

        SIM_SOPT7 &= ~(SIM_SOPT7_ADC0ALTTRGEN_MASK  |
                      SIM_SOPT7_ADC0PRETRGSEL_MASK);
        SIM_SOPT7 = SIM_SOPT7_ADC0TRGSEL(0);
        
    }
    else if(MoudelNumber == 1)//模块1
    {      
        SIM_SCGC3 |= (SIM_SCGC3_ADC1_MASK );

        SIM_SOPT7 &= ~(SIM_SOPT7_ADC1ALTTRGEN_MASK  |
                      SIM_SOPT7_ADC1PRETRGSEL_MASK) ;
        SIM_SOPT7 = SIM_SOPT7_ADC1TRGSEL(0);
           
    }
    else
    {
        return 0;
    }
    
    return 1;
}



//============================================================================
//函数名称：hw_ad_once
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//功能概要：采集一次一路模拟量的AD值    
//============================================================================
uint16 hw_ad_once(int MoudelNumber,int Channel,uint8 accuracy)//采集某路模拟量的AD值
{
    uint16 result = 0;
    //开始ADC转换
    hw_adc_convertstart(MoudelNumber, Channel, accuracy);	

    if(MoudelNumber == 0)
    {
        while (( ADC0_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK)
        {

        }
        result = ADC0_RA;        
        ADC0_SC1A &= ~ADC_SC1_COCO_MASK;
    }
    else
    {
        while (( ADC1_SC1A & ADC_SC1_COCO_MASK ) != ADC_SC1_COCO_MASK)
        {

        }
        result = ADC1_RA;        
        ADC1_SC1A &= ~ADC_SC1_COCO_MASK;
    }

    return result;
}

//============================================================================
//函数名称：hw_ad_mid
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//功能概要：中值滤波后的结果(范围:0-4095) 
//============================================================================
uint16 hw_ad_mid(int MoudelNumber,int Channel,uint8 accuracy) //中值滤波
{
	uint16 i,j,k,tmp;
	//1.取3次A/D转换结果
	i = hw_ad_once(MoudelNumber,Channel,accuracy);
	j = hw_ad_once(MoudelNumber,Channel,accuracy);
	k = hw_ad_once(MoudelNumber,Channel,accuracy);
	//2.取中值
	if (i > j)
	{
		tmp = i; i = j; j = tmp;
	}
	if (k > j) 
	  tmp = j;
	else if(k > i) 
	  tmp = k; 
    else 
      tmp = i;
	return tmp;
}

//============================================================================
//函数名称：hw_ad_ave
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//                     N:均值滤波次数(范围:0~255)
//功能概要：均值滤波后的结果(范围:0-4095) 
//============================================================================
uint16 hw_ad_ave(int MoudelNumber,int Channel,uint8 accuracy,uint8 N) //均值滤波
{
	uint32 tmp = 0;
	uint8  i;
    for(i = 0; i < N; i++)
		tmp += hw_ad_mid(MoudelNumber,Channel,accuracy);
	tmp = tmp / N; 
    return (uint16)tmp;
}

//============================================================================
//函数名称：hw_adc_convertstart
//函数返回：0 成功 ，1 失败
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//              accuracy：精度
//功能概要：开始adc转换
//============================================================================
uint8 hw_adc_convertstart(int MoudelNumber,int Channel,uint8 accuracy)
{
    uint8 ADCCfg1Mode = 0;
    
    switch(accuracy)
    {
      case 8:
        ADCCfg1Mode = 0x00;
        break;
      case 12:
        ADCCfg1Mode = 0x01;
        break;
      case 10:
        ADCCfg1Mode = 0x02;
        break;
      case 16:
        ADCCfg1Mode = 0x03;
        break;
      default:
        ADCCfg1Mode = 0x00;
    }
    
  
    //初始化ADC默认配置
    Master_Adc_Config.CONFIG1  = ADLPC_NORMAL
                              | ADC_CFG1_ADIV(ADIV_4)
                              | ADLSMP_LONG
                              | ADC_CFG1_MODE(ADCCfg1Mode)
                              | ADC_CFG1_ADICLK(ADICLK_BUS);
    Master_Adc_Config.CONFIG2  = MUXSEL_ADCA  /////
                              | ADACKEN_DISABLED
                              | ADHSC_HISPEED
                              | ADC_CFG2_ADLSTS(ADLSTS_20) ;
    Master_Adc_Config.COMPARE1 = 0x1234u ;                 //任意值
    Master_Adc_Config.COMPARE2 = 0x5678u ;                 //任意值
                                                          
    
    if(MoudelNumber == 0)
    {
        if(Channel>3&&Channel<24)                             //4~23 使用 ADC0
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(Channel);
            hw_adc_config_alt(ADC0_BASE_PTR, &Master_Adc_Config);  // 配置 ADC0
        }
        else if(Channel == 26)
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(Channel);
            hw_adc_config_alt(ADC0_BASE_PTR, &Master_Adc_Config);  // 配置ADC0
        }
        else
        {
            return 0;
        }
    }
    else
    {
        if(Channel>3&&Channel<24)                             //4~23 使用 ADC1
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(Channel);
            hw_adc_config_alt(ADC1_BASE_PTR, &Master_Adc_Config);  // 配置ADC1
        }
        else if(Channel == 26)
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(Channel);
            hw_adc_config_alt(ADC1_BASE_PTR, &Master_Adc_Config);  // 配置ADC1
        }
        else
        {
            return 0;
        }
    }
    return 1;
}
//============================================================================
//函数名称：hw_adc_config_alt
//函数返回：无符号结果值(范围:0-4095) 
//参数说明：adcmap：adc基址寄存器地址
//          ADC_CfgPtr: 存放 寄存器值的结构体    
//功能概要：将adc寄存器结构体配置进adc寄存器 
//============================================================================
void ADC_Config_Alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr)
{
    ADC_CFG1_REG(adcmap) = ADC_CfgPtr->CONFIG1;
    ADC_CFG2_REG(adcmap) = ADC_CfgPtr->CONFIG2;
    ADC_CV1_REG(adcmap)  = ADC_CfgPtr->COMPARE1; 
    ADC_CV2_REG(adcmap)  = ADC_CfgPtr->COMPARE2;
    ADC_SC2_REG(adcmap)  = ADC_CfgPtr->STATUS2;
    ADC_SC3_REG(adcmap)  = ADC_CfgPtr->STATUS3;
    ADC_PGA_REG(adcmap)  = ADC_CfgPtr->PGA;
    ADC_SC1_REG(adcmap,A)= ADC_CfgPtr->STATUS1A;       
    ADC_SC1_REG(adcmap,B)= ADC_CfgPtr->STATUS1B;
}
void hw_adc_config_alt(ADC_MemMapPtr adcmap, tADC_ConfigPtr ADC_CfgPtr)
{
    ADC_CFG1_REG(adcmap) = ADC_CfgPtr->CONFIG1;
    ADC_CFG2_REG(adcmap) = ADC_CfgPtr->CONFIG2;
    ADC_CV1_REG(adcmap)  = ADC_CfgPtr->COMPARE1; 
    ADC_CV2_REG(adcmap)  = ADC_CfgPtr->COMPARE2;
    ADC_SC2_REG(adcmap)  = ADC_CfgPtr->STATUS2;
    ADC_SC3_REG(adcmap)  = ADC_CfgPtr->STATUS3;
    ADC_PGA_REG(adcmap)  = ADC_CfgPtr->PGA;
    ADC_SC1_REG(adcmap,A)= ADC_CfgPtr->STATUS1A;       
    ADC_SC1_REG(adcmap,B)= ADC_CfgPtr->STATUS1B;
}

//============================================================================
//函数名称：hw_adc_convertstop
//函数返回：0 成功 ，1 失败
//参数说明：MoudelNumber：模块号
//               Channel：通道号
//功能概要：停止ADC转换  
//============================================================================
uint8 hw_adc_convertstop(int MoudelNumber,int Channel)
{
    if(MoudelNumber == 0)
    {
        if(Channel>3&&Channel<24)                             //4~23 使用 ADC0
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(0x1F);
            ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // 配置ADC0
        }
        else if(Channel == 26)
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(0x1F);
            ADC_Config_Alt(ADC0_BASE_PTR, &Master_Adc_Config);  // 配置ADC0
        }
        else
        {
            return 0;
        }
    }
    else if(MoudelNumber == 1)
    {
        if(Channel>3&&Channel<24)                             //4~23 使用 ADC1
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(0x1F);
            ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  //  配置ADC1
        }
        else if(Channel == 26)
        {
            Master_Adc_Config.STATUS1A = AIEN_ON | DIFF_SINGLE | ADC_SC1_ADCH(0x1F);
            ADC_Config_Alt(ADC1_BASE_PTR, &Master_Adc_Config);  //  配置ADC1
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
    return 1;
}

