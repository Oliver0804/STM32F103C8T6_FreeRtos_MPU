#include "i2c.h"
#include "delay.h"

void SDA_OUTPUT(){
  GPIO_InitTypeDef GPIO_InitStruct;
//	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_3);
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void SDA_INPUT(){
  GPIO_InitTypeDef GPIO_InitStruct;
//	HAL_GPIO_DeInit(GPIOA,GPIO_PIN_3);
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}
//MPU IIC 延時函數
void MPU_IIC_Delay(void)
{
	HAL_Delay(1);
}
//初始化IIC
void MPU_IIC_Init(void)
{
/*	
 	RCC->APB2ENR|=1<<3;		//先使能外設IO PORTB時鐘 							 
	GPIOB->CRH&=0XFFFF00FF;	//PB10/11 推輓輸出
	GPIOB->CRH|=0X00003300;	   
	GPIOB->ODR|=3<<10;     	//PB10,11 輸出高
	*/
	
}
//產生IIC起始信號
void MPU_IIC_Start(void)
{
	SDA_OUTPUT();
	//MPU_SDA_OUT();     //sda線輸出
	MPU_IIC_SDA_H;	  	  
	MPU_IIC_SCL_H;
	MPU_IIC_Delay();
 	MPU_IIC_SDA_L;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_L;//鉗住I2C總線，準備發送或接收數據 
}	  
//產生IIC停止信號
void MPU_IIC_Stop(void)
{
	SDA_OUTPUT();
	//MPU_SDA_OUT();//sda線輸出
	MPU_IIC_SCL_L;
	MPU_IIC_SDA_L;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_H; 
	MPU_IIC_SDA_H;//發送I2C總線結束信號
	MPU_IIC_Delay();							   	
}

//產生ACK應答
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
//不產生ACK應答		    
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

//等待應答信號到來
//返回值：1，接收應答失敗
//        0，接收應答成功

u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_INPUT();
	//MPU_SDA_IN();      //SDA設置為輸入  
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
	MPU_IIC_SCL_L;//時鐘輸出0 	   
	return 0;  
} 
//IIC發送一個字節
//返回從機有無應答
//1，有應答
//0，無應答			  
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUTPUT();
	//MPU_SDA_OUT(); 	    
    MPU_IIC_SCL_L;//拉低時鐘開始數據傳輸
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
//讀1個字節，ack=1時，發送ACK，ack=0，發送nACK   
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_INPUT();
	//MPU_SDA_IN();//SDA設置為輸入
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
        MPU_IIC_NAck();//發送nACK
    else
        MPU_IIC_Ack(); //發送ACK   
    return receive;
}

//IIC連續讀
//addr:器件地址
//reg:要讀取的寄存器地址
//len:要讀取的長度
//buf:讀取到的數據存儲區
//返回值:0,正常
//    其他,錯誤代碼
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//發送器件地址+寫命令	
	if(MPU_IIC_Wait_Ack())	//等待應答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//寫寄存器地址
    MPU_IIC_Wait_Ack();		//等待應答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//發送器件地址+讀命令	
    MPU_IIC_Wait_Ack();		//等待應答 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//讀數據,發送nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//讀數據,發送ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//產生一個停止條件 
	return 0;	
}
//初始化MPU6050
//返回值:0,成功
//    其他,錯誤代碼
u8 MPU_Init(void)
{ 
	u8 res;

	MPU_IIC_Init();//初始化IIC總線
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//復位MPU6050
    HAL_Delay(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//喚醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺儀傳感器,±2000dps
	MPU_Set_Accel_Fsr(3);					//加速度傳感器,±16g
	MPU_Set_Rate(500);						//設置采樣率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//關閉所有中斷
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式關閉
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//關閉FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引腳低電平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正確
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//設置CLKSEL,PLL X軸為參考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度與陀螺儀都工作
		MPU_Set_Rate(500);						//設置采樣率為50Hz
 	}else return 1;
	return 0;
}
//設置MPU6050陀螺儀傳感器滿量程範圍
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,設置成功
//    其他,設置失敗 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//設置陀螺儀滿量程範圍  
}
//設置MPU6050加速度傳感器滿量程範圍
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,設置成功
//    其他,設置失敗 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//設置加速度傳感器滿量程範圍  
}
//設置MPU6050的數字低通濾波器
//lpf:數字低通濾波頻率(Hz)
//返回值:0,設置成功
//    其他,設置失敗 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//設置數字低通濾波器  
}
//設置MPU6050的采樣率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,設置成功
//    其他,設置失敗 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//設置數字低通濾波器
 	return MPU_Set_LPF(rate/2);	//自動設置LPF為采樣率的一半
}

//得到溫度值
//返回值:溫度值(擴大了100倍)
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
//得到陀螺儀值(原始值)
//gx,gy,gz:陀螺儀x,y,z軸的原始讀數(帶符號)
//返回值:0,成功
//    其他,錯誤代碼
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
//得到加速度值(原始值)
//gx,gy,gz:陀螺儀x,y,z軸的原始讀數(帶符號)
//返回值:0,成功
//    其他,錯誤代碼
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
//IIC連續寫
//addr:器件地址 
//reg:寄存器地址
//len:寫入長度
//buf:數據區
//返回值:0,正常
//    其他,錯誤代碼
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//發送器件地址+寫命令	
	if(MPU_IIC_Wait_Ack())	//等待應答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//寫寄存器地址
    MPU_IIC_Wait_Ack();		//等待應答
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//發送數據
		if(MPU_IIC_Wait_Ack())		//等待ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 

//IIC寫一個字節 
//reg:寄存器地址
//data:數據
//返回值:0,正常
//    其他,錯誤代碼
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//發送器件地址+寫命令	
	if(MPU_IIC_Wait_Ack())	//等待應答
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//寫寄存器地址
    MPU_IIC_Wait_Ack();		//等待應答 
	MPU_IIC_Send_Byte(data);//發送數據
	if(MPU_IIC_Wait_Ack())	//等待ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IIC讀一個字節 
//reg:寄存器地址 
//返回值:讀到的數據
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//發送器件地址+寫命令	
	MPU_IIC_Wait_Ack();		//等待應答 
    MPU_IIC_Send_Byte(reg);	//寫寄存器地址
    MPU_IIC_Wait_Ack();		//等待應答
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//發送器件地址+讀命令	
    MPU_IIC_Wait_Ack();		//等待應答 
	res=MPU_IIC_Read_Byte(0);//讀取數據,發送nACK 
    MPU_IIC_Stop();			//產生一個停止條件 
	return res;		
}






