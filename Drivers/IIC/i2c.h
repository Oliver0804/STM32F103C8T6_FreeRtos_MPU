#ifndef __I2C_H
#define __I2C_H
//#include "stm32f0xx_hal.h"
#include "stm32f1xx_hal.h"

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;		   
//IO��V�]�m


//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs�H�s��,�iŪ��������,�H�s����U������
//#define MPU_PROD_ID_REG			0X0C	//prod id�H�s��,�b�H�s����U������
#define MPU_SELF_TESTX_REG		0X0D	//���˱H�s��X
#define MPU_SELF_TESTY_REG		0X0E	//���˱H�s��Y
#define MPU_SELF_TESTZ_REG		0X0F	//���˱H�s��Z
#define MPU_SELF_TESTA_REG		0X10	//���˱H�s��A
#define MPU_SAMPLE_RATE_REG		0X19	//�����W�v���W��
#define MPU_CFG_REG				0X1A	//�t�m�H�s��
#define MPU_GYRO_CFG_REG		0X1B	//�������t�m�H�s��
#define MPU_ACCEL_CFG_REG		0X1C	//�[�t�׭p�t�m�H�s��
#define MPU_MOTION_DET_REG		0X1F	//�B���˴��֭ȳ]�m�H�s��
#define MPU_FIFO_EN_REG			0X23	//FIFO�ϯ�H�s��
#define MPU_I2CMST_CTRL_REG		0X24	//IIC�D������H�s��
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC�q��0����a�}�H�s��
#define MPU_I2CSLV0_REG			0X26	//IIC�q��0�ƾڦa�}�H�s��
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC�q��0����H�s��
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC�q��1����a�}�H�s��
#define MPU_I2CSLV1_REG			0X29	//IIC�q��1�ƾڦa�}�H�s��
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC�q��1����H�s��
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC�q��2����a�}�H�s��
#define MPU_I2CSLV2_REG			0X2C	//IIC�q��2�ƾڦa�}�H�s��
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC�q��2����H�s��
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC�q��3����a�}�H�s��
#define MPU_I2CSLV3_REG			0X2F	//IIC�q��3�ƾڦa�}�H�s��
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC�q��3����H�s��
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC�q��4����a�}�H�s��
#define MPU_I2CSLV4_REG			0X32	//IIC�q��4�ƾڦa�}�H�s��
#define MPU_I2CSLV4_DO_REG		0X33	//IIC�q��4�g�ƾڱH�s��
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC�q��4����H�s��
#define MPU_I2CSLV4_DI_REG		0X35	//IIC�q��4Ū�ƾڱH�s��

#define MPU_I2CMST_STA_REG		0X36	//IIC�D�����A�H�s��
#define MPU_INTBP_CFG_REG		0X37	//���_/�Ǹ��]�m�H�s��
#define MPU_INT_EN_REG			0X38	//���_�ϯ�H�s��
#define MPU_INT_STA_REG			0X3A	//���_���A�H�s��

#define MPU_ACCEL_XOUTH_REG		0X3B	//�[�t�׭�,X�b��8��H�s��
#define MPU_ACCEL_XOUTL_REG		0X3C	//�[�t�׭�,X�b�C8��H�s��
#define MPU_ACCEL_YOUTH_REG		0X3D	//�[�t�׭�,Y�b��8��H�s��
#define MPU_ACCEL_YOUTL_REG		0X3E	//�[�t�׭�,Y�b�C8��H�s��
#define MPU_ACCEL_ZOUTH_REG		0X3F	//�[�t�׭�,Z�b��8��H�s��
#define MPU_ACCEL_ZOUTL_REG		0X40	//�[�t�׭�,Z�b�C8��H�s��

#define MPU_TEMP_OUTH_REG		0X41	//�ū׭Ȱ��K��H�s��
#define MPU_TEMP_OUTL_REG		0X42	//�ū׭ȧC8��H�s��

#define MPU_GYRO_XOUTH_REG		0X43	//��������,X�b��8��H�s��
#define MPU_GYRO_XOUTL_REG		0X44	//��������,X�b�C8��H�s��
#define MPU_GYRO_YOUTH_REG		0X45	//��������,Y�b��8��H�s��
#define MPU_GYRO_YOUTL_REG		0X46	//��������,Y�b�C8��H�s��
#define MPU_GYRO_ZOUTH_REG		0X47	//��������,Z�b��8��H�s��
#define MPU_GYRO_ZOUTL_REG		0X48	//��������,Z�b�C8��H�s��

#define MPU_I2CSLV0_DO_REG		0X63	//IIC�q��0�ƾڱH�s��
#define MPU_I2CSLV1_DO_REG		0X64	//IIC�q��1�ƾڱH�s��
#define MPU_I2CSLV2_DO_REG		0X65	//IIC�q��2�ƾڱH�s��
#define MPU_I2CSLV3_DO_REG		0X66	//IIC�q��3�ƾڱH�s��

#define MPU_I2CMST_DELAY_REG	0X67	//IIC�D�����ɺ޲z�H�s��
#define MPU_SIGPATH_RST_REG		0X68	//�H���q�D�_��H�s��
#define MPU_MDETECT_CTRL_REG	0X69	//�B���˴�����H�s��
#define MPU_USER_CTRL_REG		0X6A	//�Τᱱ��H�s��
#define MPU_PWR_MGMT1_REG		0X6B	//�q���޲z�H�s��1
#define MPU_PWR_MGMT2_REG		0X6C	//�q���޲z�H�s��2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO�p�ƱH�s�����K��
#define MPU_FIFO_CNTL_REG		0X73	//FIFO�p�ƱH�s���C�K��
#define MPU_FIFO_RW_REG			0X74	//FIFOŪ�g�H�s��
#define MPU_DEVICE_ID_REG		0X75	//����ID�H�s��
 
#define MPU_ADDR				0X68


//#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
//#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}

//IO�ާ@���	 
#define MPU_IIC_SCL_H     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);		//SCL
#define MPU_IIC_SCL_L      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);		//SCL
#define MPU_IIC_SDA_H     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);		//SDA
#define MPU_IIC_SDA_L      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);		//SDA
#define MPU_READ_SDA     HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)

//IIC�Ҧ��ާ@���
void MPU_IIC_Delay(void);				//MPU IIC���ɨ��
void MPU_IIC_Init(void);                //��l��IIC��IO�f				 
void MPU_IIC_Start(void);				//�o�eIIC�}�l�H��
void MPU_IIC_Stop(void);	  			//�o�eIIC����H��
void MPU_IIC_Send_Byte(u8 txd);			//IIC�o�e�@�Ӧr�`
u8   MPU_IIC_Read_Byte(unsigned char ack);//IICŪ���@�Ӧr�`
u8   MPU_IIC_Wait_Ack(void); 				//IIC����ACK�H��
void MPU_IIC_Ack(void);					//IIC�o�eACK�H��
void MPU_IIC_NAck(void);				//IIC���o�eACK�H��

void IMPU_IC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 MPU_IIC_Read_One_Byte(u8 daddr,u8 addr);	  



u8 MPU_Init(void); 								//��l��MPU6050
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//IIC�s��g
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); //IIC�s��Ū 
u8 MPU_Write_Byte(u8 reg,u8 data);				//IIC�g�@�Ӧr�`
u8 MPU_Read_Byte(u8 reg);						//IICŪ�@�Ӧr�`

u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_Fifo(u8 sens);


short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif
















