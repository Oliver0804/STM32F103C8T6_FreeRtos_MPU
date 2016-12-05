#include "stm32f0xx_hal.h"

#define  SCL_SET() 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)
#define  SCL_RESET() 	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)
#define SDA_SET() 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#define  SDA_RESET() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)
#define  READ_SDA() HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)

// [ b] [ I] [ I] [ b] [ / I] [ I] [ b] [ / I] [ I] [ b] [ / I] [ I] [ b] [ / I] [ I] [ b] [ / I]以下是模擬I2C函數[ / b] [ / b] [ / b] [ / b] [ / b] [ / b] [ / I] * **************// 
/* ================================ ================================ 
【名稱】無效I2CDelay(unsigned char型T)
【功能】模擬IIC用短的延時
【注備】
【。作者】
【時間】
================================== ============================== */ 
void I2CDelay (unsigned char t)
{
while(t--);
}

/* ================================================== ============== 
【名稱】無效I2CINIT(無效)
【功能】I2C初始化,狀態空閒
【注備】
【。作者】
【時間】
====== ================================================== ======== */ 
void I2CInit(void)
{
SDA_SET();
SCL_SET();
}



/* ======================= ========================================= 
【名稱】無效I2CStart(無效)
【功能】I2C信號起始
【注備】SCL,SDA同為高,SDA跳變成低之後,SCL變成跳低
【。作者】
【時間】
=========== ================================================== === */ 
void I2CStart(void)
{
SDA_SET();
    SCL_SET();
I2CDelay(10);
SDA_RESET();
I2CDelay(20);
SCL_RESET();
I2CDelay(20);
}
/* ===== ================================================== ========= 
【名稱】無效I2CStop(無效)
【功能】I2C信號停止
【注備】SCL,SDA同為低,SCL跳變成高之後,SDA變成跳高
【。作】者
【時間】
============================================ ==================== */ 
void I2CStop(void)
{
SDA_RESET();
SCL_RESET();
I2CDelay(10);
SCL_SET();
I2CDelay(10);
SDA_SET();
I2CDelay(10);
}




/* ====================================== ========================== 
【名稱】unsigned char型I2CWRByte(unsigned char型WRBYTE)
【功能】I2C寫一個字節數據,返回ACK或者NACK 
【備注】從高到低,發送依次
【。作者】
【時間】
============================ ==================================== */ 
unsigned char I2CWRByte(unsigned char WRByte)
{
unsigned char i;
SCL_RESET();
for(i=0;i<8;i++)
{
    if(WRByte&0x80)
    {
        SDA_SET();  
    }
    else
    {
        SDA_RESET();
    }
    I2CDelay(10);
    SCL_SET();      //?出SDA?定后,拉高SCL?出上升沿,?机??到后?行?据采?
    I2CDelay(50);
    SCL_RESET();
    I2CDelay(10);
    WRByte <<= 1;
} 
SDA_SET();  
SCL_SET();
I2CDelay(20);
if(READ_SDA()==1)           //SDA?高,收到NACK
{
    SCL_RESET();
    I2CDelay(50);
    return 1;    
}
else                //SDA?低,收到ACK
{
    SCL_RESET();
    I2CDelay(50);
    return 0;
}
}


/* ================================== ============================== 
【名稱】unsigned char型I2CRDByte(unsigned char型AckValue)
【功能】I2C讀一個字節數據,入口參數用於控制應答狀態,ACK或者NACK 
【備注】從高到低,接收依次
【。作者】
【時間】
================= =============================================== */ 
unsigned char I2CRDByte(unsigned char AckValue)
{
unsigned char i,RDByte=0;
SCL_RESET();
SDA_SET();          //?放??  
for (i=0;i<8;i++) 
{
    RDByte <<= 1; //移位
    SCL_SET();      //?出上升沿
    I2CDelay(30);   //延?等待信??定
    if(READ_SDA()==1)       //采??取?据
    {
        RDByte |= 0x01;
    }
    else
    {
        RDByte &= 0xfe;
    }
    SCL_RESET();        //下降沿,?机?出下一位值
    I2CDelay(10);
} 
if(AckValue )//?答??
{
     SDA_SET();
}
else
{
    SDA_RESET();
}

I2CDelay(10);
SCL_SET();                         
I2CDelay(50);          
SCL_RESET(); 
SDA_SET();              
I2CDelay(1);
return RDByte;
}