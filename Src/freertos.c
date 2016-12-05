/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "delay.h"
#include <math.h>
#include "main.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId Fall_TaskHandle;
osThreadId Read_IIC_TaskHandle;
osThreadId UART_TaskHandle;

/* USER CODE BEGIN Variables */
	float pitch,roll,yaw; 		//歐拉角
	short aacx,aacy,aacz;		//加速度傳感器原始數據
	short gyrox,gyroy,gyroz;	//陀螺儀原始數據
	short temp;					//溫度
	int s;


	
#define Kp 100.0f                        // 比例增益支配率收?到加速度?/磁??
#define Ki 0.002f                // ?分增益支配率的陀螺?偏?的?接
#define halfT 0.001f                // 采?周期的一半
#define AVG_VAL 5 
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元?的元素，代表估?方向
float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例?小?分?差
float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻?角
float ACCEL_XOUT; //x軸加速度值暫存
float ACCEL_YOUT; //y軸加速度值暫存
float ACCEL_ZOUT; //z軸加速度值暫存
float GYRO_XOUT;  //x軸陀螺儀值暫存
float GYRO_YOUT;  //y軸陀螺儀值暫存
float GYRO_ZOUT;  //z軸陀螺儀值暫存
uint8_t Post_Enable;
uint8_t Falls_count=0;
uint8_t Falls_status=0;
uint32_t SF=22000;
uint32_t SW=10000;
uint32_t ST=30;
uint32_t SC=30;
uint32_t SA=75;
uint32_t FC=87;
uint32_t FS=78;
	int test_map=0;
	long  ACCEL_ARRAY[AVG_VAL]={0};
	long  GYRO_ARRAY[AVG_VAL]={0};
	long ACCEL_AVG;
	long GYRO_AVG;
	long ACCEL_AVG_OK;
	long GYRO_AVG_OK;
		char TX_TEMP[100]={0x00};
extern volatile struct UART_Interface_s BlueTooth_Uart;
extern volatile struct UART_Interface_s Zigbee_Uart;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void Start_Fall_Task(void const * argument);
void Start_Read_IIC(void const * argument);
void Start_UART_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
uint8_t UART_IS_IDLE(uint32_t Timeout_ms);
	
int my_Atoi(uint8_t * s,int size){
	 int count=0;
	 int sun=0;
	 for(count=0;count<size;count++){
		//sun=sun+(s[count]-0x30)*(10^(size-count));
		 sun=sun+(s[count]-0x30)*(pow(10,(size-count-1)));
	 }
		return sun;
 }

void HAL_GPIO_LED(GPIO_TypeDef* GPIOx, uint16_t Led_no, char LedState){
	  /* Check the parameters */

if(Led_no==1){
		if(LedState == 'G')
		{
			HAL_GPIO_WritePin(GPIOx, PS_LED_G_Pin ,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOx, PS_LED_R_Pin ,GPIO_PIN_SET);
		}
		else if(LedState == 'N')
		{
			HAL_GPIO_WritePin(GPIOx, PS_LED_G_Pin ,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOx, PS_LED_R_Pin ,GPIO_PIN_SET);
			
		}else if(LedState =='R')
		{
			HAL_GPIO_WritePin(GPIOx, PS_LED_G_Pin ,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOx, PS_LED_R_Pin ,GPIO_PIN_RESET);
		}
	}else if(Led_no==2){
			if(LedState == 'G')
		{
			HAL_GPIO_WritePin(GPIOx, SGN_LED_G_Pin ,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOx, SGN_LED_R_Pin ,GPIO_PIN_SET);
		}
		else if(LedState == 'N')
		{
			HAL_GPIO_WritePin(GPIOx, SGN_LED_G_Pin ,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOx, SGN_LED_R_Pin ,GPIO_PIN_SET);
		}else if(LedState =='R')
		{
			HAL_GPIO_WritePin(GPIOx, SGN_LED_G_Pin ,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOx, SGN_LED_R_Pin ,GPIO_PIN_RESET);
		}
	}
}
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Fall_Task */
  osThreadDef(Fall_Task, Start_Fall_Task, osPriorityNormal, 0, 128);
  Fall_TaskHandle = osThreadCreate(osThread(Fall_Task), NULL);

  /* definition and creation of Read_IIC_Task */
  osThreadDef(Read_IIC_Task, Start_Read_IIC, osPriorityIdle, 0, 128);
  Read_IIC_TaskHandle = osThreadCreate(osThread(Read_IIC_Task), NULL);

  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, Start_UART_Task, osPriorityIdle, 0, 128);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* Start_Fall_Task function */
void Start_Fall_Task(void const * argument)
{

  /* USER CODE BEGIN Start_Fall_Task */
  /* Infinite loop */
	 uint16_t TIME_OUT=30*100;
	uint16_t TIMER=0;
	uint16_t SCORE=500;
	uint16_t WARNING=ST-20;
  for(;;)
  {
   		if(HAL_GPIO_ReadPin(GPIOB,BOTTOM_1_Pin)==GPIO_PIN_RESET){
			osDelay(5);
		}
		if(HAL_GPIO_ReadPin(GPIOB,BOTTOM_2_Pin)==GPIO_PIN_RESET){
			HAL_GPIO_WritePin(GPIOB, PS_LED_G_Pin ,GPIO_PIN_SET);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOB, PS_LED_G_Pin ,GPIO_PIN_RESET);
		  osDelay(100);
		}
		
		osDelay(5);
		if(FS==1){
			if(WARNING>1){
			WARNING--;
			//osDelay(1);
			}else{
				sprintf(TX_TEMP,"@BT+FS=%d#",FS);
			  BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				WARNING=ST;
			}
			HAL_GPIO_WritePin(GPIOB, SGN_LED_R_Pin ,GPIO_PIN_SET);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOB, SGN_LED_R_Pin ,GPIO_PIN_RESET);
		  osDelay(100);
		}else{
			HAL_GPIO_WritePin(GPIOB, PS_LED_R_Pin ,GPIO_PIN_SET);
			osDelay(100);
			HAL_GPIO_WritePin(GPIOB, PS_LED_R_Pin ,GPIO_PIN_RESET);
		  osDelay(100);
		}
		if(ACCEL_AVG_OK>=SF){
			SCORE=500;
	    HAL_GPIO_WritePin(GPIOB, PS_LED_R_Pin ,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOB, PS_LED_G_Pin ,GPIO_PIN_SET);
			while(TIMER<TIME_OUT){
				osDelay(1);
				if(SCORE>1000){
					FS=1;
				break;
				}
				if(GYRO_AVG_OK<=500){//停止
				SCORE++;
				}else if(GYRO_AVG_OK>500&&(GYRO_AVG_OK<=1500)){//移動
				if(SCORE>0){
					SCORE--;
				}
				}else if(GYRO_AVG_OK>1500){//衝擊
					SCORE=0;
					FS=0;
					break;
			}
		}
     //osDelay(1000);
		}else{
			HAL_GPIO_LED(GPIOB,1,'N');
		}
  }
  /* USER CODE END Start_Fall_Task */
}

/* Start_Read_IIC function */
void Start_Read_IIC(void const * argument)
{
  /* USER CODE BEGIN Start_Read_IIC */
	MPU_Init();
  /* Infinite loop */
  for(;;)
  {
		  temp=MPU_Get_Temperature();	//得到溫度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度傳感器數據
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺儀數據
			ACCEL_XOUT=aacx; //x軸加速度值暫存
			ACCEL_YOUT=aacy; //y軸加速度值暫存
			ACCEL_ZOUT=aacz; //z軸加速度值暫存
			GYRO_XOUT=gyrox;  //x軸陀螺儀值暫存
			GYRO_YOUT=gyroy;  //y軸陀螺儀值暫存
			GYRO_ZOUT=gyroz;  //z軸陀螺儀值暫存
	  	ACCEL_AVG_OK=sqrt(ACCEL_XOUT*ACCEL_XOUT+ACCEL_YOUT*ACCEL_YOUT+ACCEL_ZOUT*ACCEL_ZOUT)*10;
	   osDelay(1);
	   GYRO_AVG_OK=sqrt(GYRO_XOUT*GYRO_XOUT+GYRO_YOUT*GYRO_YOUT+GYRO_ZOUT*GYRO_ZOUT);
		 osDelay(1);
  }
  /* USER CODE END Start_Read_IIC */
}

/* Start_UART_Task function */
void Start_UART_Task(void const * argument)
{
  /* USER CODE BEGIN Start_UART_Task */
	
/*
BlueTooth_Uart.Sender((uint8_t*)"OK",0);
//strcmp(A,B)==0;
strcpy(pString + 1, Buffer1);包含NULL結尾)
memcpy(pString + 1, Buffer2, 3);不包含NULL結尾)
*/
	uint8_t BUFF_SCAN_FLAG=0;
	uint8_t BUFF_CMD_FLAG=0;
	uint8_t S_FLAG=0; //@
	uint8_t PLUS_FLAG=0;  //+
	uint8_t EQUAL_FLAG=0;  //=
  uint8_t COMMA_FLAG=0;  //,
	uint8_t E_FLAG=0;  //#
  uint8_t ASK_FLAG=0;  //?
	uint8_t ERR_FLAG=0;
	char TEMP[50]={0x00};

	char CMD_DEVICE[10]={0x00};
	char CMD_ACTION[10]={0x00};
	char CMD_VAL_1[9]={0x00};
	int INT_VAL_1=0;
  char CMD_VAL_2[9]={0x00};
	//BlueTooth_Uart.Sender((uint8_t*)"SYS OK!",0);					
  for(;;)
  {

		//HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_RESET);
    if(BlueTooth_Uart.RX_leng > 0)
    {
		//HAL_GPIO_WritePin(GPIOC, LD3_Pin, GPIO_PIN_SET);

      if(UART_IS_IDLE(BlueTooth_Uart.RX_Idle))
      {
				BUFF_SCAN_FLAG=0;
				S_FLAG=0xFF; //@
				PLUS_FLAG=0xFF;  //+
	      EQUAL_FLAG=0xFF;  //=
        COMMA_FLAG=0xFF;  //,
				E_FLAG=0xFF;  //#
				ASK_FLAG=0xFF;//?
				 while(BUFF_SCAN_FLAG<BlueTooth_Uart.RX_leng+1){
					 osDelay(1);
					 if(BlueTooth_Uart.RX_Buffer[BUFF_SCAN_FLAG]=='@' ) {
							S_FLAG=BUFF_SCAN_FLAG;
					  }
					  else if(BlueTooth_Uart.RX_Buffer[BUFF_SCAN_FLAG]=='+' ) {
							PLUS_FLAG=BUFF_SCAN_FLAG;	
						}
						 else if(BlueTooth_Uart.RX_Buffer[BUFF_SCAN_FLAG]=='=' ) {
							EQUAL_FLAG=BUFF_SCAN_FLAG;	
						}
						  else if(BlueTooth_Uart.RX_Buffer[BUFF_SCAN_FLAG]=='#' ) {
							E_FLAG=BUFF_SCAN_FLAG;	
						}
							else if(BlueTooth_Uart.RX_Buffer[BUFF_SCAN_FLAG]==',' ) {
							COMMA_FLAG=BUFF_SCAN_FLAG;	
						}else if(BlueTooth_Uart.RX_Buffer[BUFF_SCAN_FLAG]=='?' ) {
							ASK_FLAG=BUFF_SCAN_FLAG;	
						}
							else if(S_FLAG!=0xFF&&E_FLAG!=0xFF){
							BUFF_CMD_FLAG=0;
							for(int COUNT=S_FLAG+1;COUNT<PLUS_FLAG;COUNT++){
								CMD_DEVICE[BUFF_CMD_FLAG]=BlueTooth_Uart.RX_Buffer[COUNT];
								CMD_DEVICE[BUFF_CMD_FLAG+1]=0x00;
								BUFF_CMD_FLAG++;								
							}
							BUFF_CMD_FLAG=0;
							for(int COUNT=PLUS_FLAG+1;COUNT<EQUAL_FLAG;COUNT++){
								CMD_ACTION[BUFF_CMD_FLAG]=BlueTooth_Uart.RX_Buffer[COUNT];
								CMD_ACTION[BUFF_CMD_FLAG+1]=0x00;
								BUFF_CMD_FLAG++;								
							}
							if(COMMA_FLAG!=0xFF){
								BUFF_CMD_FLAG=0;
								for(int COUNT=EQUAL_FLAG+1;COUNT<COMMA_FLAG;COUNT++){
										CMD_VAL_1[BUFF_CMD_FLAG]=BlueTooth_Uart.RX_Buffer[COUNT];
										CMD_VAL_1[BUFF_CMD_FLAG+1]=0x00;
										BUFF_CMD_FLAG++;								
									}
								BUFF_CMD_FLAG=0;
								for(int COUNT=COMMA_FLAG+1;COUNT<E_FLAG;COUNT++){
										CMD_VAL_2[BUFF_CMD_FLAG]=BlueTooth_Uart.RX_Buffer[COUNT];
										CMD_VAL_2[BUFF_CMD_FLAG+1]=0x00;
										BUFF_CMD_FLAG++;								
									}
								}else{
										BUFF_CMD_FLAG=0;
										for(int COUNT=EQUAL_FLAG+1;COUNT<E_FLAG;COUNT++){
										CMD_VAL_1[BUFF_CMD_FLAG]=BlueTooth_Uart.RX_Buffer[COUNT];
										CMD_VAL_1[BUFF_CMD_FLAG+1]=0x00;
										BUFF_CMD_FLAG++;								
									}
								}
						}else{
						//BlueTooth_Uart.Sender((uint8_t*)"@BT+CMD=ERR#",0);	
						}
						BUFF_SCAN_FLAG++;
				 }
				 BlueTooth_Uart.RX_leng=0;
      }
    }else{
		  
		}
    
    if(Zigbee_Uart.RX_leng > 0)
    {
     if(UART_IS_IDLE(Zigbee_Uart.RX_Idle))
      {
    }    
  }
		osDelay(1);
		if(strcmp(CMD_DEVICE,"BT")==0){
			if(strcmp(CMD_ACTION,"SF")==0){
				if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SF);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SF=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"SW")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SW);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SW=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"SC")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SC);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SC=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"ST")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,ST);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  ST=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"SA")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SA);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SA=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"FS")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,FS);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"ERROR");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  FS=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"FC")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,FC);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"ERROR");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  FC=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"GS")==0){
					if(ASK_FLAG!=0xFF){
						/*
						float ACCEL_XOUT; //x軸加速度值暫存
						float ACCEL_YOUT; //y軸加速度值暫存
						float ACCEL_ZOUT; //z軸加速度值暫存
						float GYRO_XOUT;  //x軸陀螺儀值暫存
						float GYRO_YOUT;  //y軸陀螺儀值暫存
						float GYRO_ZOUT;  //z軸陀螺儀值暫存
						*/
					sprintf(TX_TEMP,"@%s+%s=%f,%f,%f,%f,%f,%f,0,0,0#",CMD_DEVICE,CMD_ACTION,ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT,GYRO_XOUT,GYRO_YOUT,GYRO_ZOUT);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"ERROR");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  FC=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}
			CMD_DEVICE[0]=0;	
		}else if(strcmp(CMD_DEVICE,"ZB")==0){
						if(strcmp(CMD_ACTION,"SF")==0){
				if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SF);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SF=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"SW")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SW);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SW=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"SC")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SC);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SC=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"ST")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,ST);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  ST=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"SA")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,SA);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"OK");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  SA=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"FS")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,FS);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"ERROR");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  FS=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}else if(strcmp(CMD_ACTION,"FC")==0){
					if(ASK_FLAG!=0xFF){
					sprintf(TX_TEMP,"@%s+%s=%d#",CMD_DEVICE,CMD_ACTION,FC);
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				}else{
					sprintf(TX_TEMP,"@%s+%s=%s#",CMD_DEVICE,CMD_ACTION,"ERROR");
			    BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
				  FC=my_Atoi((uint8_t *)CMD_VAL_1,strlen(CMD_VAL_1));
				}
			}
			CMD_DEVICE[0]=0;				
		}
		/*
		if(strcmp(CMD_TEMP,"BT+GG")==0){
		  BlueTooth_Uart.Sender((uint8_t*)"*",0);	
			CMD_TEMP[1]=0x00;
			osDelay(2);
		}else if(strcmp(CMD_TEMP,"BT+SF=?")==0){
			BlueTooth_Uart.Sender((uint8_t*)"@BT+SF=",0);	
			sprintf(TX_TEMP,"%2d",SF);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			BlueTooth_Uart.Sender((uint8_t*)"#",0);	
			CMD_TEMP[1]=0x00;	
		}else if(strcmp(CMD_TEMP,"BT+SW=?")==0){
			BlueTooth_Uart.Sender((uint8_t*)"@BT+SW=",0);	
			sprintf(TX_TEMP,"%2d",SW);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			BlueTooth_Uart.Sender((uint8_t*)"#",0);	
			CMD_TEMP[1]=0x00;				
		}else if(strcmp(CMD_TEMP,"BT+SC=?")==0){
			BlueTooth_Uart.Sender((uint8_t*)"@BT+SC=",0);	
			sprintf(TX_TEMP,"%2d",SC);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			BlueTooth_Uart.Sender((uint8_t*)"#",0);	
			CMD_TEMP[1]=0x00;				
		}else if(strcmp(CMD_TEMP,"BT+ST=?")==0){
			BlueTooth_Uart.Sender((uint8_t*)"@BT+ST=",0);	
			sprintf(TX_TEMP,"%2d",ST);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			BlueTooth_Uart.Sender((uint8_t*)"#",0);	
			CMD_TEMP[1]=0x00;				
		}else if(strcmp(CMD_TEMP,"BT+SA=?")==0){
			BlueTooth_Uart.Sender((uint8_t*)"@BT+SA=",0);	
			sprintf(TX_TEMP,"%2d",SA);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			BlueTooth_Uart.Sender((uint8_t*)"#",0);	
			CMD_TEMP[1]=0x00;		
		}else if(strcmp(CMD_TEMP,"BT+GS=?")==0){
			BlueTooth_Uart.Sender((uint8_t*)"@BT+GS=",0);	
			sprintf(TX_TEMP,"%0.0f,%0.0f,%0.0f,",ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			sprintf(TX_TEMP,"%0.0f,%0.0f,%0.0f",GYRO_XOUT,GYRO_YOUT,GYRO_ZOUT);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			BlueTooth_Uart.Sender((uint8_t*)"#",0);	
			CMD_TEMP[1]=0x00;
		}else if(strcmp(CMD_TEMP,"BT+TS=?")==0){
			BlueTooth_Uart.Sender((uint8_t*)"@BT+TS=",0);	
			sprintf(TX_TEMP,"%2d",ACCEL_AVG_OK);
			BlueTooth_Uart.Sender((uint8_t*)TX_TEMP,0);	
			BlueTooth_Uart.Sender((uint8_t*)"#",0);	
			CMD_TEMP[1]=0x00;	
		}
		*/

		osDelay(10);
  }
  /* USER CODE END Start_UART_Task */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
