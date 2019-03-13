#include "kalman.h"
#include "MPU6050.h"
#include "arm_math.h"
#include "math.h"

float32_t Accel_x;	     //X����ٶ�ֵ�ݴ�
float32_t Accel_y;	     //Y����ٶ�ֵ�ݴ�
float32_t Accel_z;	     //Z����ٶ�ֵ�ݴ�

float32_t Gyro_x;		 //X�������������ݴ�
float32_t Gyro_y;        //Y�������������ݴ�
float32_t Gyro_z;		 //Z�������������ݴ�

//float32_t Angle_gy;    //�ɽ��ٶȼ������б�Ƕ�
float32_t Angle_x_temp;  //�ɼ��ٶȼ����x��б�Ƕ�
float32_t Angle_y_temp;  //�ɼ��ٶȼ����y��б�Ƕ�

float32_t Angle_X_Final; //X������б�Ƕ�
float32_t Angle_Y_Final; //Y������б�Ƕ�

//�Ƕȼ���
void Angle_Calcu(void)	 
{
	MPU6050_RawDataStructTypeDef tmp;

	//��ΧΪ2gʱ�������ϵ��16384 LSB/g
	//deg = rad*180/3.14

	float32_t x,y,z;
	tmp = HiSTM_MPU6050_Get_RawData();

	Accel_x = (float32_t) tmp.Accel_X;
	Accel_y = (float32_t) tmp.Accel_Y;
	Accel_z = (float32_t) tmp.Accel_Z;
	Gyro_x  = (float32_t) tmp.Gyro_X;
	Gyro_y  = (float32_t) tmp.Gyro_Y;
	Gyro_z  = (float32_t) tmp.Gyro_Z;
	
	//����x����ٶ�
	if(Accel_x<32764) x=Accel_x/16384;
	else              x=1-(Accel_x-49152)/16384;
	
	//����y����ٶ�
	if(Accel_y<32764) y=Accel_y/16384;
	else              y=1-(Accel_y-49152)/16384;
	
	//����z����ٶ�
	if(Accel_z<32764) z=Accel_z/16384;
	else              z=(Accel_z-49152)/16384;

	//�ü��ٶȼ����������ˮƽ������ϵ֮��ļн�
	Angle_x_temp = (atan(y / z)) * 180 / 3.14;
	Angle_y_temp = (atan(x / z)) * 180 / 3.14;

	//�Ƕȵ�������											
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
	if(Accel_z<32764) {}
	if(Accel_z>32764) {}
	
	//���ٶ�
	//��ǰ�˶�
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/32.8);//��ΧΪ1000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
	//����˶�
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/32.8;
	//��ǰ�˶�
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/32.8);//��ΧΪ1000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
	//����˶�
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/32.8;
	//��ǰ�˶�
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/32.8);//��ΧΪ1000deg/sʱ�������ϵ��16.4 LSB/(deg/s)
	//����˶�
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/32.8;
	
	//Angle_gy = Angle_gy + Gyro_y*0.025;  //���ٶȻ��ֵõ���б�Ƕ�.Խ����ֳ����ĽǶ�Խ��
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //�������˲�����Y���
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //�������˲�����Y���
															  
} 


//����������		
float32_t  Q_angle = 0.001;
float32_t  Q_gyro  = 0.003;
float32_t  R_angle = 0.5;
float32_t  dt      = 0.01;//dtΪkalman�˲�������ʱ��;
int8_t     C_0     = 1;
float32_t  Q_bias, Angle_err;
float32_t  PCt_0, PCt_1, E;
float32_t  K_0, K_1, t_0, t_1;
float32_t  Pdot[4] ={0, 0, 0, 0};
float32_t  PP[2][2] = { { 1, 0 }, { 0, 1 } };

void Kalman_Filter_X(float32_t Accel,float32_t Gyro) //����������
{
	Angle_X_Final += (Gyro - Q_bias) * dt; //�������
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_X_Final += K_0 * Angle_err;	 //�������
	Q_bias        += K_1 * Angle_err;	 //�������
	Gyro_x         = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

void Kalman_Filter_Y(float32_t Accel,float32_t Gyro) //����������
{
	Angle_Y_Final += (Gyro - Q_bias) * dt; //�������
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��

	Pdot[1]=- PP[1][1];
	Pdot[2]=- PP[1][1];
	Pdot[3]=Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // =����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	Gyro_y   = Gyro - Q_bias;	 //���ֵ(�������)��΢��=���ٶ�
}

