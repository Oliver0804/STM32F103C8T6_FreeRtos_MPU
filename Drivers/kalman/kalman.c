#include "kalman.h"

#include "math.h"

int ii=0;
int jj=0;
float Accel_x;	     //X�b�[�t�׭ȼȦs
float Accel_y;	     //Y�b�[�t�׭ȼȦs
float Accel_z;	     //Z�b�[�t�׭ȼȦs

float Gyro_x;		 //X�b�������ƾڼȦs
float Gyro_y;        //Y�b�������ƾڼȦs
float Gyro_z;		 //Z�b�������ƾڼȦs

//float Angle_gy;    //�Ѩ��t�׭p�⪺�ɱר���
float Angle_x_temp;  //�ѥ[�t�׭p�⪺x�ɱר���
float Angle_y_temp;  //�ѥ[�t�׭p�⪺y�ɱר���

float Angle_X_Final; //X�̲׶ɱר���
float Angle_Y_Final; //Y�̲׶ɱר���

extern float ACCEL_XOUT; //x�b�[�t�׭ȼȦs
extern float ACCEL_YOUT; //y�b�[�t�׭ȼȦs
extern float ACCEL_ZOUT; //z�b�[�t�׭ȼȦs
extern float GYRO_XOUT;  //x�b�������ȼȦs
extern float GYRO_YOUT;  //y�b�������ȼȦs
extern float GYRO_ZOUT;  //z�b�������ȼȦs
//���׭p��
void Angle_Calcu(void)	 
{
	//�d��2g�ɡA�������Y�G16384 LSB/g
	//deg = rad*180/3.14
	float x,y,z;   
		//IMUupdate(gyrox,gyroy,gyroz,aacx,aacy,aacz);
	Accel_x = ACCEL_XOUT; //x�b�[�t�׭ȼȦs
	Accel_y = ACCEL_YOUT; //y�b�[�t�׭ȼȦs
	Accel_z = ACCEL_ZOUT; //z�b�[�t�׭ȼȦs
	Gyro_x  =GYRO_XOUT;  //x�b�������ȼȦs
	Gyro_y  = GYRO_YOUT;  //y�b�������ȼȦs
	Gyro_z  = GYRO_ZOUT;  //z�b�������ȼȦs
	
	//�B�zx�b�[�t��
	if(Accel_x<32764) x=Accel_x/16384;
	else              x=1-(Accel_x-49152)/16384;
	
	//�B�zy�b�[�t��     
	if(Accel_y<32764) y=Accel_y/16384;
	else              y=1-(Accel_y-49152)/16384;
	
	//�B�zz�b�[�t��
	if(Accel_z<32764) z=Accel_z/16384;
	else              z=(Accel_z-49152)/16384;

	//�Υ[�t�׭p��T�Ӷb�M���������Шt����������
	Angle_x_temp=(atan(y/z))*180/3.14;
	Angle_y_temp=(atan(x/z))*180/3.14;

	//���ת����t��											
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
	if(Accel_z<32764) {}
	if(Accel_z>32764) {}
	
	//���t��
	//�V�e�B��
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);//�d��1000deg/s�ɡA�������Y�G16.4 LSB/(deg/s)
	//�V��B��
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
	//�V�e�B��
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);//�d��1000deg/s�ɡA�������Y�G16.4 LSB/(deg/s)
	//�V��B��
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
	//�V�e�B��
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);//�d��1000deg/s�ɡA�������Y�G16.4 LSB/(deg/s)
	//�V��B��
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
	
	//Angle_gy = Angle_gy + Gyro_y*0.025;  //���t�׿n���o��ɱר���.�V�j�n���X�Ӫ����׶V�j
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //�d�����o�i�p��Y�ɨ�
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //�d�����o�i�p��Y�ɨ�
															  
} 


//�d���ҰѼ�		
float Q_angle = 0.001;  
float Q_gyro  = 0.003;
float R_angle = 0.5;
float dt      = 0.01;//dt��kalman�o�i�����ˮɶ�;
char  C_0     = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro) //�d���Ҩ��		
{
	ii=ii+1;
        Angle_X_Final += (Gyro - Q_bias) * dt; //������p
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-������p�~�t���t���L��

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-������p�~�t���t�L�����n��
	PP[0][1] += Pdot[1] * dt;   // =������p�~�t���t
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-������p
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //������p�~�t���t
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_X_Final += K_0 * Angle_err;	 //������p
        if(ii==10)
        {ii=0;
        printf("\r\n X��V���ƾ�:%4.2f ��",Angle_X_Final);
        }
	Q_bias        += K_1 * Angle_err;	 //������p
	Gyro_x         = Gyro - Q_bias;	 //��X��(������p)���L��=���t��
}

void Kalman_Filter_Y(float Accel,float Gyro) //�d���Ҩ��		
{
	
        jj=jj+1;
        Angle_Y_Final += (Gyro - Q_bias) * dt; //������p
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-������p�~�t���t���L��

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-������p�~�t���t�L�����n��
	PP[0][1] += Pdot[1] * dt;   // =������p�~�t���t
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-������p
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //������p�~�t���t
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //������p
        if(jj==10)
        {
          jj=0;
        printf("\r\n Y��V���ƾ�:%4.2f ��",Angle_Y_Final);
					
	printf("\r\n //////////////////////////////////////////r\n");
        }
        Q_bias	+= K_1 * Angle_err;	 //������p
	Gyro_y   = Gyro - Q_bias;	 //��X��(������p)���L��=���t��
}

