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

//PID参数结构体
typedef struct{
	float kp;         //p系数
	float kd;         //d系数
	float ki;         //i系数
	float expect_kd;  //期望值d系数
	float fb_kd;      //反馈值d系数
} PID_ARG_structure;

//PID数据结构体
typedef struct{
	float expect_old; //上次的期望值
	float fb_old;     //上次的反馈值
	float err_old;    //上次的误差
	float out;        //输出
	float err_i;      //误差积分
	
	float err;        //调试用 确认PID每一项的大小
	float err_d;
	float expect_d;
	float fb_d;
} PID_VAL_structure;

float my_sqrt_reciprocal(float number);
float my_sqrt(float number);
float my_abs(float f);
REAL fast_atan2(REAL y, REAL x);
void pid_calcu(float dT_s,                 //周期
	           float expect,               //期望值
			   float feedback,             //反馈值
			   PID_ARG_structure *pid_arg, //pid参数结构体
			   PID_VAL_structure *pid_val, //pid数据结构体
			   float err_i_lim,            //积分限幅
				u8 inter_en);            //积分使能
double mx_sin(double rad);
double my_sin(double rad);
float my_cos(double rad);
float my_deadzone(float x,float medi,float zoom);
#endif
