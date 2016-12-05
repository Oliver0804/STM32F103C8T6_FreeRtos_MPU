#include "i2c.h"
#include "delay.h"
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
void SDA_OUTPUT(){
  GPIO_InitTypeDef GPIO_InitStruct;
//	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_3);
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void SDA_INPUT(){
  GPIO_InitTypeDef GPIO_InitStruct;
//	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_3);
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
//MPU IIC ���ɨ��
void MPU_IIC_Delay(void)
{
	HAL_Delay(1);
}
//��l��IIC
void MPU_IIC_Init(void)
{
/*	
 	RCC->APB2ENR|=1<<3;		//���ϯ�~�]IO PORTB���� 							 
	GPIOB->CRH&=0XFFFF00FF;	//PB10/11 ������X
	GPIOB->CRH|=0X00003300;	   
	GPIOB->ODR|=3<<10;     	//PB10,11 ��X��
	*/
	
}
//����IIC�_�l�H��
void MPU_IIC_Start(void)
{
	SDA_OUTPUT();
	//MPU_SDA_OUT();     //sda�u��X
	MPU_IIC_SDA_H;	  	  
	MPU_IIC_SCL_H;
	MPU_IIC_Delay();
 	MPU_IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_L;//�X��I2C�`�u�A�ǳƵo�e�α����ƾ� 
}	  
//����IIC����H��
void MPU_IIC_Stop(void)
{
	SDA_OUTPUT();
	//MPU_SDA_OUT();//sda�u��X
	MPU_IIC_SCL_L;
	MPU_IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_H; 
	MPU_IIC_SDA_H;//�o�eI2C�`�u�����H��
	MPU_IIC_Delay();							   	
}

//����ACK����
void MPU_IIC_Ack(void)
{
	MPU_IIC_SCL_L;
	SDA_OUTPUT();
	//MPU_SDA_OUT();
	MPU_IIC_SDA_L;
	MPU_IIC_Delay();
	MPU_IIC_SCL_H;
	MPU_IIC_Delay();
	MPU_IIC_SCL_L;
}
//������ACK����		    
void MPU_IIC_NAck(void)
{
	MPU_IIC_SCL_L;
	SDA_OUTPUT();
	//MPU_SDA_OUT();
	MPU_IIC_SDA_H;
	MPU_IIC_Delay();
	MPU_IIC_SCL_H;
	MPU_IIC_Delay();
	MPU_IIC_SCL_L;
}	

//���������H�����
//��^�ȡG1�A������������
//        0�A�����������\

u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_INPUT();
	//MPU_SDA_IN();      //SDA�]�m����J  
	MPU_IIC_SDA_H;
	MPU_IIC_Delay();	   
	MPU_IIC_SCL_H;
	MPU_IIC_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	MPU_IIC_SCL_L;//������X0 	   
	return 0;  
} 
//IIC�o�e�@�Ӧr�`
//��^�q�����L����
//1�A������
//0�A�L����			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUTPUT();
	//MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_L;//�ԧC�����}�l�ƾڶǿ�
    for(t=0;t<8;t++)
    {   
			if((txd&0x80)>>7==1){
			MPU_IIC_SDA_H;
			}else{
			MPU_IIC_SDA_L;
			}
        txd<<=1; 	  
		MPU_IIC_SCL_H;
		MPU_IIC_Delay(); 
		MPU_IIC_SCL_L;	
		MPU_IIC_Delay();
    }	 
} 	    
//Ū1�Ӧr�`�Aack=1�ɡA�o�eACK�Aack=0�A�o�enACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_INPUT();
	//MPU_SDA_IN();//SDA�]�m����J
    for(i=0;i<8;i++ )
	{
        MPU_IIC_SCL_L; 
        MPU_IIC_Delay();
		    MPU_IIC_SCL_H;
        receive<<=1;
        if(MPU_READ_SDA==GPIO_PIN_SET)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//�o�enACK
    else
        MPU_IIC_Ack(); //�o�eACK   
    return receive;
}

//IIC�s��Ū
//addr:����a�}
//reg:�nŪ�����H�s���a�}
//len:�nŪ��������
//buf:Ū���쪺�ƾڦs�x��
//��^��:0,���`
//    ��L,���~�N�X
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//�o�e����a�}+�g�R�O	
	if(MPU_IIC_Wait_Ack())	//��������
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//�g�H�s���a�}
    MPU_IIC_Wait_Ack();		//��������
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//�o�e����a�}+Ū�R�O	
    MPU_IIC_Wait_Ack();		//�������� 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//Ū�ƾ�,�o�enACK 
		else *buf=MPU_IIC_Read_Byte(1);		//Ū�ƾ�,�o�eACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//���ͤ@�Ӱ������ 
	return 0;	
}
//��l��MPU6050
//��^��:0,���\
//    ��L,���~�N�X
u8 MPU_Init(void)
{ 
	u8 res;

	MPU_IIC_Init();//��l��IIC�`�u
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//�_��MPU6050
    HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//���MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�������ǷP��,��2000dps
	MPU_Set_Accel_Fsr(3);					//�[�t�׶ǷP��,��16g
	MPU_Set_Rate(50);						//�]�m���˲v50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�����Ҧ����_
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C�D�Ҧ�����
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//����FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT�޸}�C�q������
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID���T
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//�]�mCLKSEL,PLL X�b���Ѧ�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//�[�t�׻P���������u�@
		MPU_Set_Rate(50);						//�]�m���˲v��50Hz
 	}else return 1;
	return 0;
}
//�]�mMPU6050�������ǷP�����q�{�d��
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//��^��:0,�]�m���\
//    ��L,�]�m���� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//�]�m���������q�{�d��  
}
//�]�mMPU6050�[�t�׶ǷP�����q�{�d��
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//��^��:0,�]�m���\
//    ��L,�]�m���� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//�]�m�[�t�׶ǷP�����q�{�d��  
}
//�]�mMPU6050���Ʀr�C�q�o�i��
//lpf:�Ʀr�C�q�o�i�W�v(Hz)
//��^��:0,�]�m���\
//    ��L,�]�m���� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�]�m�Ʀr�C�q�o�i��  
}
//�]�mMPU6050�����˲v(���wFs=1KHz)
//rate:4~1000(Hz)
//��^��:0,�]�m���\
//    ��L,�]�m���� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�]�m�Ʀr�C�q�o�i��
 	return MPU_Set_LPF(rate/2);	//�۰ʳ]�mLPF�����˲v���@�b
}

//�o��ū׭�
//��^��:�ū׭�(�X�j�F100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�o���������(��l��)
//gx,gy,gz:������x,y,z�b����lŪ��(�a�Ÿ�)
//��^��:0,���\
//    ��L,���~�N�X
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//�o��[�t�׭�(��l��)
//gx,gy,gz:������x,y,z�b����lŪ��(�a�Ÿ�)
//��^��:0,���\
//    ��L,���~�N�X
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC�s��g
//addr:����a�} 
//reg:�H�s���a�}
//len:�g�J����
//buf:�ƾڰ�
//��^��:0,���`
//    ��L,���~�N�X
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//�o�e����a�}+�g�R�O	
	if(MPU_IIC_Wait_Ack())	//��������
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//�g�H�s���a�}
    MPU_IIC_Wait_Ack();		//��������
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//�o�e�ƾ�
		if(MPU_IIC_Wait_Ack())		//����ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 

//IIC�g�@�Ӧr�` 
//reg:�H�s���a�}
//data:�ƾ�
//��^��:0,���`
//    ��L,���~�N�X
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//�o�e����a�}+�g�R�O	
	if(MPU_IIC_Wait_Ack())	//��������
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//�g�H�s���a�}
    MPU_IIC_Wait_Ack();		//�������� 
	MPU_IIC_Send_Byte(data);//�o�e�ƾ�
	if(MPU_IIC_Wait_Ack())	//����ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IICŪ�@�Ӧr�` 
//reg:�H�s���a�} 
//��^��:Ū�쪺�ƾ�
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//�o�e����a�}+�g�R�O	
	MPU_IIC_Wait_Ack();		//�������� 
    MPU_IIC_Send_Byte(reg);	//�g�H�s���a�}
    MPU_IIC_Wait_Ack();		//��������
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//�o�e����a�}+Ū�R�O	
    MPU_IIC_Wait_Ack();		//�������� 
	res=MPU_IIC_Read_Byte(0);//Ū���ƾ�,�o�enACK 
    MPU_IIC_Stop();			//���ͤ@�Ӱ������ 
	return res;		
}






