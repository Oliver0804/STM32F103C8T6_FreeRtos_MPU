#include "kalman.h"

#include "math.h"

int ii=0;
int jj=0;
float Accel_x;	     //X軸加速度值暫存
float Accel_y;	     //Y軸加速度值暫存
float Accel_z;	     //Z軸加速度值暫存

float Gyro_x;		 //X軸陀螺儀數據暫存
float Gyro_y;        //Y軸陀螺儀數據暫存
float Gyro_z;		 //Z軸陀螺儀數據暫存

//float Angle_gy;    //由角速度計算的傾斜角度
float Angle_x_temp;  //由加速度計算的x傾斜角度
float Angle_y_temp;  //由加速度計算的y傾斜角度

float Angle_X_Final; //X最終傾斜角度
float Angle_Y_Final; //Y最終傾斜角度

extern float ACCEL_XOUT; //x軸加速度值暫存
extern float ACCEL_YOUT; //y軸加速度值暫存
extern float ACCEL_ZOUT; //z軸加速度值暫存
extern float GYRO_XOUT;  //x軸陀螺儀值暫存
extern float GYRO_YOUT;  //y軸陀螺儀值暫存
extern float GYRO_ZOUT;  //z軸陀螺儀值暫存
//角度計算
void Angle_Calcu(void)	 
{
	//範圍為2g時，換算關係：16384 LSB/g
	//deg = rad*180/3.14
	float x,y,z;   
		//IMUupdate(gyrox,gyroy,gyroz,aacx,aacy,aacz);
	Accel_x = ACCEL_XOUT; //x軸加速度值暫存
	Accel_y = ACCEL_YOUT; //y軸加速度值暫存
	Accel_z = ACCEL_ZOUT; //z軸加速度值暫存
	Gyro_x  =GYRO_XOUT;  //x軸陀螺儀值暫存
	Gyro_y  = GYRO_YOUT;  //y軸陀螺儀值暫存
	Gyro_z  = GYRO_ZOUT;  //z軸陀螺儀值暫存
	
	//處理x軸加速度
	if(Accel_x<32764) x=Accel_x/16384;
	else              x=1-(Accel_x-49152)/16384;
	
	//處理y軸加速度     
	if(Accel_y<32764) y=Accel_y/16384;
	else              y=1-(Accel_y-49152)/16384;
	
	//處理z軸加速度
	if(Accel_z<32764) z=Accel_z/16384;
	else              z=(Accel_z-49152)/16384;

	//用加速度計算三個軸和水平面坐標系之間的夾角
	Angle_x_temp=(atan(y/z))*180/3.14;
	Angle_y_temp=(atan(x/z))*180/3.14;

	//角度的正負號											
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
	if(Accel_z<32764) {}
	if(Accel_z>32764) {}
	
	//角速度
	//向前運動
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);//範圍為1000deg/s時，換算關係：16.4 LSB/(deg/s)
	//向後運動
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
	//向前運動
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);//範圍為1000deg/s時，換算關係：16.4 LSB/(deg/s)
	//向後運動
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
	//向前運動
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);//範圍為1000deg/s時，換算關係：16.4 LSB/(deg/s)
	//向後運動
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
	
	//Angle_gy = Angle_gy + Gyro_y*0.025;  //角速度積分得到傾斜角度.越大積分出來的角度越大
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //卡爾曼濾波計算Y傾角
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //卡爾曼濾波計算Y傾角
															  
} 


//卡爾曼參數		
float Q_angle = 0.001;  
float Q_gyro  = 0.003;
float R_angle = 0.5;
float dt      = 0.01;//dt為kalman濾波器采樣時間;
char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro) //卡爾曼函數		
{
	ii=ii+1;
        Angle_X_Final += (Gyro - Q_bias) * dt; //先驗估計
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先驗估計誤差協方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先驗估計誤差協方差微分的積分
	PP[0][1] += Pdot[1] * dt;   // =先驗估計誤差協方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-先驗估計
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //後驗估計誤差協方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_X_Final += K_0 * Angle_err;	 //後驗估計
        if(ii==10)
        {ii=0;
        printf("\r\n X方向的數據:%4.2f 度",Angle_X_Final);
        }
	Q_bias        += K_1 * Angle_err;	 //後驗估計
	Gyro_x         = Gyro - Q_bias;	 //輸出值(後驗估計)的微分=角速度
}

void Kalman_Filter_Y(float Accel,float Gyro) //卡爾曼函數		
{
	
        jj=jj+1;
        Angle_Y_Final += (Gyro - Q_bias) * dt; //先驗估計
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先驗估計誤差協方差的微分

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先驗估計誤差協方差微分的積分
	PP[0][1] += Pdot[1] * dt;   // =先驗估計誤差協方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-先驗估計
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //後驗估計誤差協方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //後驗估計
        if(jj==10)
        {
          jj=0;
        printf("\r\n Y方向的數據:%4.2f 度",Angle_Y_Final);
					
	printf("\r\n //////////////////////////////////////////r\n");
        }
        Q_bias	+= K_1 * Angle_err;	 //後驗估計
	Gyro_y   = Gyro - Q_bias;	 //輸出值(後驗估計)的微分=角速度
}

