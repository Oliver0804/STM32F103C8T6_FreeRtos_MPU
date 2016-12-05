#ifndef __I2C_H
#define __I2C_H
//#include "stm32f0xx_hal.h"
#include "stm32f1xx_hal.h"

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;		   
//IO方向設置


//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可讀取版本號,寄存器手冊未提到
//#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手冊未提到
#define MPU_SELF_TESTX_REG		0X0D	//自檢寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自檢寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自檢寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自檢寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采樣頻率分頻器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺儀配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度計配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//運動檢測閥值設置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主機控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC從機0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC從機0數據地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC從機0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC從機1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC從機1數據地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC從機1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC從機2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC從機2數據地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC從機2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC從機3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC從機3數據地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC從機3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC從機4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC從機4數據地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC從機4寫數據寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC從機4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC從機4讀數據寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主機狀態寄存器
#define MPU_INTBP_CFG_REG		0X37	//中斷/旁路設置寄存器
#define MPU_INT_EN_REG			0X38	//中斷使能寄存器
#define MPU_INT_STA_REG			0X3A	//中斷狀態寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X軸高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X軸低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y軸高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y軸低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z軸高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z軸低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//溫度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//溫度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺儀值,X軸高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺儀值,X軸低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺儀值,Y軸高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺儀值,Y軸低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺儀值,Z軸高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺儀值,Z軸低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC從機0數據寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC從機1數據寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC從機2數據寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC從機3數據寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主機延時管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信號通道復位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//運動檢測控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用戶控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//電源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//電源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO計數寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO計數寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO讀寫寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
 
#define MPU_ADDR				0X68


//#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
//#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO操作函數	 
#define MPU_IIC_SCL_H     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);		//SCL
#define MPU_IIC_SCL_L      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);		//SCL
#define MPU_IIC_SDA_H     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);		//SDA
#define MPU_IIC_SDA_L      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);		//SDA
#define MPU_READ_SDA     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)

//IIC所有操作函數
void MPU_IIC_Delay(void);				//MPU IIC延時函數
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);				//發送IIC開始信號
void MPU_IIC_Stop(void);	  			//發送IIC停止信號
void MPU_IIC_Send_Byte(u8 txd);			//IIC發送一個字節
u8   MPU_IIC_Read_Byte(unsigned char ack);//IIC讀取一個字節
u8   MPU_IIC_Wait_Ack(void); 				//IIC等待ACK信號
void MPU_IIC_Ack(void);					//IIC發送ACK信號
void MPU_IIC_NAck(void);				//IIC不發送ACK信號

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  



u8 MPU_Init(void); 								//初始化MPU6050
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//IIC連續寫
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); //IIC連續讀 
u8 MPU_Write_Byte(u8 reg,u8 data);				//IIC寫一個字節
u8 MPU_Read_Byte(u8 reg);						//IIC讀一個字節

u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_Fifo(u8 sens);


short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif
















