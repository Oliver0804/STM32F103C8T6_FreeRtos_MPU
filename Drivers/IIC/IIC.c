#include "stm32f0xx_hal.h"

#define  SCL_SET() 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET)
#define  SCL_RESET() 	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET)
#define SDA_SET() 			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET)
#define  SDA_RESET() HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET)
#define  READ_SDA() HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)

// [ b] [ I] [ I] [ b] [ / I] [ I] [ b] [ / I] [ I] [ b] [ / I] [ I] [ b] [ / I] [ I] [ b] [ / I]�H�U�O����I2C���[ / b] [ / b] [ / b] [ / b] [ / b] [ / b] [ / I] * **************// 
/* ================================ ================================ 
�i�W�١j�L��I2CDelay(unsigned char��T)
�i�\��j����IIC�εu������
�i�`�ơj
�i�C�@�̡j
�i�ɶ��j
================================== ============================== */ 
void I2CDelay (unsigned char t)
{
while(t--);
}

/* ================================================== ============== 
�i�W�١j�L��I2CINIT(�L��)
�i�\��jI2C��l��,���A�Ŷ�
�i�`�ơj
�i�C�@�̡j
�i�ɶ��j
====== ================================================== ======== */ 
void I2CInit(void)
{
SDA_SET();
SCL_SET();
}



/* ======================= ========================================= 
�i�W�١j�L��I2CStart(�L��)
�i�\��jI2C�H���_�l
�i�`�ơjSCL,SDA�P����,SDA���ܦ��C����,SCL�ܦ����C
�i�C�@�̡j
�i�ɶ��j
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
�i�W�١j�L��I2CStop(�L��)
�i�\��jI2C�H������
�i�`�ơjSCL,SDA�P���C,SCL���ܦ�������,SDA�ܦ�����
�i�C�@�j��
�i�ɶ��j
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
�i�W�١junsigned char��I2CWRByte(unsigned char��WRBYTE)
�i�\��jI2C�g�@�Ӧr�`�ƾ�,��^ACK�Ϊ�NACK 
�i�ƪ`�j�q����C,�o�e�̦�
�i�C�@�̡j
�i�ɶ��j
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
    SCL_SET();      //?�XSDA?�w�Z,�԰�SCL?�X�W�ɪu,?��??��Z?��?�u��?
    I2CDelay(50);
    SCL_RESET();
    I2CDelay(10);
    WRByte <<= 1;
} 
SDA_SET();  
SCL_SET();
I2CDelay(20);
if(READ_SDA()==1)           //SDA?��,����NACK
{
    SCL_RESET();
    I2CDelay(50);
    return 1;    
}
else                //SDA?�C,����ACK
{
    SCL_RESET();
    I2CDelay(50);
    return 0;
}
}


/* ================================== ============================== 
�i�W�١junsigned char��I2CRDByte(unsigned char��AckValue)
�i�\��jI2CŪ�@�Ӧr�`�ƾ�,�J�f�ѼƥΩ󱱨��������A,ACK�Ϊ�NACK 
�i�ƪ`�j�q����C,�����̦�
�i�C�@�̡j
�i�ɶ��j
================= =============================================== */ 
unsigned char I2CRDByte(unsigned char AckValue)
{
unsigned char i,RDByte=0;
SCL_RESET();
SDA_SET();          //?��??  
for (i=0;i<8;i++) 
{
    RDByte <<= 1; //����
    SCL_SET();      //?�X�W�ɪu
    I2CDelay(30);   //��?���ݫH??�w
    if(READ_SDA()==1)       //��??��?�u
    {
        RDByte |= 0x01;
    }
    else
    {
        RDByte &= 0xfe;
    }
    SCL_RESET();        //�U���u,?��?�X�U�@���
    I2CDelay(10);
} 
if(AckValue )//?��??
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