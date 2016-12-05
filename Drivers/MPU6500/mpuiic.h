#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//���{�ǥu�ѾǲߨϥΡA���g�@�̳\�i�A���o�Ω�䥦����γ~
//ALIENTEK��ĥSTM32�}�o�OV3
//MPU6050 IIC�X�� �N�X	   
//���I��l@ALIENTEK
//�޳N�׾�:www.openedv.com
//�Ыؤ��:2015/1/17
//�����GV1.0
//���v�Ҧ��A�s�����s�C
//Copyright(C) �s�{���P�l�q�l��ަ������q 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 	   		   
//IO��V�]�m
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO�ާ@���	 
#define MPU_IIC_SCL    PBout(10) 		//SCL
#define MPU_IIC_SDA    PBout(11) 		//SDA	 
#define MPU_READ_SDA   PBin(11) 		//��JSDA 

//IIC�Ҧ��ާ@���
void MPU_IIC_Delay(void);				//MPU IIC���ɨ��
void MPU_IIC_Init(void);                //��l��IIC��IO�f				 
void MPU_IIC_Start(void);				//�o�eIIC�}�l�H��
void MPU_IIC_Stop(void);	  			//�o�eIIC����H��
void MPU_IIC_Send_Byte(u8 txd);			//IIC�o�e�@�Ӧr�`
u8 MPU_IIC_Read_Byte(unsigned char ack);//IICŪ���@�Ӧr�`
u8 MPU_IIC_Wait_Ack(void); 				//IIC����ACK�H��
void MPU_IIC_Ack(void);					//IIC�o�eACK�H��
void MPU_IIC_NAck(void);				//IIC���o�eACK�H��

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  
#endif
















