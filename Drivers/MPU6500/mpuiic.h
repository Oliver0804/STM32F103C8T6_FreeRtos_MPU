#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供學習使用，未經作者許可，不得用於其它任何用途
//ALIENTEK戰艦STM32開發板V3
//MPU6050 IIC驅動 代碼	   
//正點原子@ALIENTEK
//技術論壇:www.openedv.com
//創建日期:2015/1/17
//版本：V1.0
//版權所有，盜版必究。
//Copyright(C) 廣州市星翼電子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 	   		   
//IO方向設置
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO操作函數	 
#define MPU_IIC_SCL    PBout(10) 		//SCL
#define MPU_IIC_SDA    PBout(11) 		//SDA	 
#define MPU_READ_SDA   PBin(11) 		//輸入SDA 

//IIC所有操作函數
void MPU_IIC_Delay(void);				//MPU IIC延時函數
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//發送IIC開始信號
void MPU_IIC_Stop(void);	  			//發送IIC停止信號
void MPU_IIC_Send_Byte(u8 txd);			//IIC發送一個字節
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC讀取一個字節
u8 MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信號
void MPU_IIC_Ack(void);					//IIC發送ACK信號
void MPU_IIC_NAck(void);				//IIC不發送ACK信號

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















