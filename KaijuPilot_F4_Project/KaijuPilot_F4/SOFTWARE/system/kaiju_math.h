#ifndef __KAIJU_MATH_H__
#define __KAIJU_MATH_H__

//#include "io.h"
#include "stm32f4xx.h"

#define REAL float
#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define TAN_MAP_SIZE    256

#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define my_pow(a) ((a)*(a))

//PID�����ṹ��
typedef struct{
	float kp;         //pϵ��
	float kd;         //dϵ��
	float ki;         //iϵ��
	float expect_kd;  //����ֵdϵ��
	float fb_kd;      //����ֵdϵ��
} PID_ARG_structure;

//PID���ݽṹ��
typedef struct{
	float expect_old; //�ϴε�����ֵ
	float fb_old;     //�ϴεķ���ֵ
	float err_old;    //�ϴε����
	float out;        //���
	float err_i;      //������
	
	float err;        //������ ȷ��PIDÿһ��Ĵ�С
	float err_d;
	float expect_d;
	float fb_d;
} PID_VAL_structure;

float my_sqrt_reciprocal(float number);
float my_sqrt(float number);
float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
void pid_calcu(float dT_s,                 //����
	           float expect,               //����ֵ
			   float feedback,             //����ֵ
			   PID_ARG_structure *pid_arg, //pid�����ṹ��
			   PID_VAL_structure *pid_val, //pid���ݽṹ��
			   float err_i_lim,            //�����޷�
				u8 inter_en);            //����ʹ��
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deadzone(float x,float medi,float zoom);
#endif
