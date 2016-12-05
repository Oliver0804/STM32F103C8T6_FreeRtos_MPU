 #include "myadc.h"
 #include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK戰艦STM32開發板
//ADC 代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//修改日期:2012/9/7
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
	   
		   
//初始化ADC
//這裡我們僅以規則通道為例
//我們默認將開啟通道0~3																	   
void  Adc_Init(void)
{ 	
	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道時鐘
 

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //設置ADC分頻因子6 72M/6=12,ADC最大時間不能超過14M

	//PA1 作為模擬通道輸入引腳                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模擬輸入引腳
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	ADC_DeInit(ADC1);  //復位ADC1 

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在獨立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模數轉換工作在單通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模數轉換工作在單次轉換模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//轉換由軟件而不是外部觸發啟動
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC數據右對齊
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//順序進行規則轉換的ADC通道的數目
	ADC_Init(ADC1, &ADC_InitStructure);	//根據ADC_InitStruct中指定的參數初始化外設ADCx的寄存器   

  
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能復位校準  
	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待復位校準結束
	
	ADC_StartCalibration(ADC1);	 //開啟AD校準
 
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校準結束
 
//	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的軟件轉換啟動功能

}				  
//獲得ADC值
//ch:通道值 0~3
u16 Get_Adc(u8 ch)   
{
  	//設置指定ADC的規則組通道，一個序列，采樣時間
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采樣時間為239.5週期	  			    
  
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的軟件轉換啟動功能	
	 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待轉換結束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1規則組的轉換結果
}

u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 	 



























