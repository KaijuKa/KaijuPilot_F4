#include "kaiju_math.h"

const float fast_atan_table[257] = 
{
	0.000000e+00, 3.921549e-03, 7.842976e-03, 1.176416e-02,
	1.568499e-02, 1.960533e-02, 2.352507e-02, 2.744409e-02,
	3.136226e-02, 3.527947e-02, 3.919560e-02, 4.311053e-02,
	4.702413e-02, 5.093629e-02, 5.484690e-02, 5.875582e-02,
	6.266295e-02, 6.656816e-02, 7.047134e-02, 7.437238e-02,
	7.827114e-02, 8.216752e-02, 8.606141e-02, 8.995267e-02,
	9.384121e-02, 9.772691e-02, 1.016096e-01, 1.054893e-01,
	1.093658e-01, 1.132390e-01, 1.171087e-01, 1.209750e-01,
	1.248376e-01, 1.286965e-01, 1.325515e-01, 1.364026e-01,
	1.402496e-01, 1.440924e-01, 1.479310e-01, 1.517652e-01,
	1.555948e-01, 1.594199e-01, 1.632403e-01, 1.670559e-01,
	1.708665e-01, 1.746722e-01, 1.784728e-01, 1.822681e-01,
	1.860582e-01, 1.898428e-01, 1.936220e-01, 1.973956e-01,
	2.011634e-01, 2.049255e-01, 2.086818e-01, 2.124320e-01,
	2.161762e-01, 2.199143e-01, 2.236461e-01, 2.273716e-01,
	2.310907e-01, 2.348033e-01, 2.385093e-01, 2.422086e-01,
	2.459012e-01, 2.495869e-01, 2.532658e-01, 2.569376e-01,
	2.606024e-01, 2.642600e-01, 2.679104e-01, 2.715535e-01,
	2.751892e-01, 2.788175e-01, 2.824383e-01, 2.860514e-01,
	2.896569e-01, 2.932547e-01, 2.968447e-01, 3.004268e-01,
	3.040009e-01, 3.075671e-01, 3.111252e-01, 3.146752e-01,
	3.182170e-01, 3.217506e-01, 3.252758e-01, 3.287927e-01,
	3.323012e-01, 3.358012e-01, 3.392926e-01, 3.427755e-01,
	3.462497e-01, 3.497153e-01, 3.531721e-01, 3.566201e-01,
	3.600593e-01, 3.634896e-01, 3.669110e-01, 3.703234e-01,
	3.737268e-01, 3.771211e-01, 3.805064e-01, 3.838825e-01,
	3.872494e-01, 3.906070e-01, 3.939555e-01, 3.972946e-01,
	4.006244e-01, 4.039448e-01, 4.072558e-01, 4.105574e-01,
	4.138496e-01, 4.171322e-01, 4.204054e-01, 4.236689e-01,
	4.269229e-01, 4.301673e-01, 4.334021e-01, 4.366272e-01,
	4.398426e-01, 4.430483e-01, 4.462443e-01, 4.494306e-01,
	4.526070e-01, 4.557738e-01, 4.589307e-01, 4.620778e-01,
	4.652150e-01, 4.683424e-01, 4.714600e-01, 4.745676e-01,
	4.776654e-01, 4.807532e-01, 4.838312e-01, 4.868992e-01,
	4.899573e-01, 4.930055e-01, 4.960437e-01, 4.990719e-01,
	5.020902e-01, 5.050985e-01, 5.080968e-01, 5.110852e-01,
	5.140636e-01, 5.170320e-01, 5.199904e-01, 5.229388e-01,
	5.258772e-01, 5.288056e-01, 5.317241e-01, 5.346325e-01,
	5.375310e-01, 5.404195e-01, 5.432980e-01, 5.461666e-01,
	5.490251e-01, 5.518738e-01, 5.547124e-01, 5.575411e-01,
	5.603599e-01, 5.631687e-01, 5.659676e-01, 5.687566e-01,
	5.715357e-01, 5.743048e-01, 5.770641e-01, 5.798135e-01,
	5.825531e-01, 5.852828e-01, 5.880026e-01, 5.907126e-01,
	5.934128e-01, 5.961032e-01, 5.987839e-01, 6.014547e-01,
	6.041158e-01, 6.067672e-01, 6.094088e-01, 6.120407e-01,
	6.146630e-01, 6.172755e-01, 6.198784e-01, 6.224717e-01,
	6.250554e-01, 6.276294e-01, 6.301939e-01, 6.327488e-01,
	6.352942e-01, 6.378301e-01, 6.403565e-01, 6.428734e-01,
	6.453808e-01, 6.478788e-01, 6.503674e-01, 6.528466e-01,
	6.553165e-01, 6.577770e-01, 6.602282e-01, 6.626701e-01,
	6.651027e-01, 6.675261e-01, 6.699402e-01, 6.723452e-01,
	6.747409e-01, 6.771276e-01, 6.795051e-01, 6.818735e-01,
	6.842328e-01, 6.865831e-01, 6.889244e-01, 6.912567e-01,
	6.935800e-01, 6.958943e-01, 6.981998e-01, 7.004964e-01,
	7.027841e-01, 7.050630e-01, 7.073330e-01, 7.095943e-01,
	7.118469e-01, 7.140907e-01, 7.163258e-01, 7.185523e-01,
	7.207701e-01, 7.229794e-01, 7.251800e-01, 7.273721e-01,
	7.295557e-01, 7.317307e-01, 7.338974e-01, 7.360555e-01,
	7.382053e-01, 7.403467e-01, 7.424797e-01, 7.446045e-01,
	7.467209e-01, 7.488291e-01, 7.509291e-01, 7.530208e-01,
	7.551044e-01, 7.571798e-01, 7.592472e-01, 7.613064e-01,
	7.633576e-01, 7.654008e-01, 7.674360e-01, 7.694633e-01,
	7.714826e-01, 7.734940e-01, 7.754975e-01, 7.774932e-01,
	7.794811e-01, 7.814612e-01, 7.834335e-01, 7.853983e-01,
	7.853983e-01
};

/*******************************************************************************
* 函 数 名         : my_sqrt_reciprocal
* 函数功能		     : 计算开根号后的倒数
* 输    入         : float
* 输    出         : 无
*******************************************************************************/
float my_sqrt_reciprocal(float number)
{
	long i;
	float x, y;

	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( 1.5f - ( x * y * y ) );
	y = y * ( 1.5f - ( x * y * y ) );
	
	return y;
}

/*******************************************************************************
* 函 数 名         : my_sqrt
* 函数功能		     : 计算平方根
* 输    入         : float
* 输    出         : 无
*******************************************************************************/
float my_sqrt(float number)
{
	return number * my_sqrt_reciprocal(number);
}

/*******************************************************************************
* 函 数 名         : my_abs
* 函数功能		     : 求绝对值
* 输    入         : float
* 输    出         : 无
*******************************************************************************/
float my_abs(float f)
{
	if (f >= 0.0f)
	{
		return f;
	}

	return -f;
}

/*******************************************************************************
* 函 数 名         : fast_atan2
* 函数功能		     : 快速计算角度
* 输    入         : float
* 输    出         : 无
*******************************************************************************/
REAL fast_atan2(REAL y, REAL x) 
{
	REAL x_abs, y_abs, z;
	REAL alpha, angle, base_angle;
	int index;

	/* don't divide by zero! */
	if ((y == 0.0f) && (x == 0.0f))
		angle = 0.0f;
	else 
	{
		/* normalize to +/- 45 degree range */
		y_abs = my_abs(y);
		x_abs = my_abs(x);
		//z = (y_abs < x_abs ? y_abs / x_abs : x_abs / y_abs);
		if (y_abs < x_abs)
			z = y_abs / x_abs;
		else
			z = x_abs / y_abs;
		/* when ratio approaches the table resolution, the angle is */
		/*      best approximated with the argument itself...       */
		if (z < TAN_MAP_RES)
			base_angle = z;
		else 
		{
			/* find index and interpolation value */
			alpha = z * (REAL) TAN_MAP_SIZE - .5f;
			index = (int) alpha;
			alpha -= (REAL) index;
			/* determine base angle based on quadrant and */
			/* add or subtract table value from base angle based on quadrant */
			base_angle = fast_atan_table[index];
			base_angle += (fast_atan_table[index + 1] - fast_atan_table[index]) * alpha;
		}

		if (x_abs > y_abs) 
		{        /* -45 -> 45 or 135 -> 225 */
			if (x >= 0.0f) 
			{           /* -45 -> 45 */
				if (y >= 0.0f)
					angle = base_angle;   /* 0 -> 45, angle OK */
				else
					angle = -base_angle;  /* -45 -> 0, angle = -angle */
			} 
			else
			{                  /* 135 -> 180 or 180 -> -135 */
				angle = 3.14159265358979323846;

				if (y >= 0.0f)
					angle -= base_angle;  /* 135 -> 180, angle = 180 - angle */
				else
					angle = base_angle - angle;   /* 180 -> -135, angle = angle - 180 */
			}
		} 
		else 
		{                    /* 45 -> 135 or -135 -> -45 */
			if (y >= 0.0f) 
			{           /* 45 -> 135 */
				angle = 1.57079632679489661923;

				if (x >= 0.0f)
					angle -= base_angle;  /* 45 -> 90, angle = 90 - angle */
				else
					angle += base_angle;  /* 90 -> 135, angle = 90 + angle */
			} 
			else
			{                  /* -135 -> -45 */
				angle = -1.57079632679489661923;

				if (x >= 0.0f)
					angle += base_angle;  /* -90 -> -45, angle = -90 + angle */
				else
					angle -= base_angle;  /* -135 -> -90, angle = -90 - angle */
			}
		}
	}


#ifdef ZERO_TO_TWOPI
	if (angle < 0)
		return (angle + TWOPI);
	else
		return (angle);
#else
	return (angle);
#endif
}

/*******************************************************************************
* 函 数 名         : pid_calcu
* 函数功能		     : PID计算
* 输    入         : ......
* 输    出         : 无
*******************************************************************************/
void pid_calcu(float dT_s,                 //周期
	           float expect,               //期望值
			   float feedback,             //反馈值
			   PID_ARG_structure *pid_arg, //pid参数结构体
			   PID_VAL_structure *pid_val, //pid数据结构体
			   float err_i_lim,            //积分限幅
				u8 inter_en)            //积分使能
{
	float err, err_d, expect_d, hz, fb_d;
	hz = safe_div(1.0f, dT_s, 0);
	
	//计算误差 误差的微分 期望值的微分 反馈值的微分
	fb_d = (pid_val->fb_old - feedback) * hz;
	expect_d = (pid_val->expect_old - expect) * hz;
	err = (expect + expect_d * pid_arg->expect_kd) - (feedback + fb_d * pid_arg->fb_kd);
	err_d = (pid_val->err_old - err) * hz;
	
	//计算误差积分 并限幅
	pid_val->err_i += pid_arg->ki * err * dT_s;
	pid_val->err_i = LIMIT(pid_val->err_i, -err_i_lim, err_i_lim) *  inter_en;
	
	//计算结果并更新pid_val
	pid_val->out = err * pid_arg->kp
					+ err_d * pid_arg->kd
					+ pid_val->err_i;
	
	//调试用
	pid_val->err = err * pid_arg->kp;
	pid_val->err_d = err_d * pid_arg->kd;
	pid_val->expect_d = expect_d * pid_arg->expect_kd;
	pid_val->fb_d = fb_d * pid_arg->fb_kd;
	
	pid_val->err_old = err;
	pid_val->expect_old = expect;
}

#define ONE_PI   (3.14159265)

/*******************************************************************************
* 函 数 名         : mx_sin
* 函数功能		     : 计算sin函数1
* 输    入         : 角度rad
* 输    出         : sin
*******************************************************************************/
double mx_sin(double rad)
{
	double sine;
	if (rad < 0)
		sine = rad*(1.27323954f + 0.405284735f * rad);
	else
		sine = rad * (1.27323954f - 0.405284735f * rad);
	if (sine < 0)
		sine = sine*(-0.225f * (sine + 1) + 1);
	else
		sine = sine * (0.225f *( sine - 1) + 1);
	return sine;
}

/*******************************************************************************
* 函 数 名         : my_sin
* 函数功能		     : 计算sin函数2
* 输    入         : 角度rad
* 输    出         : sin
*******************************************************************************/
double my_sin(double rad)
{
	s8 flag_ = 1;

	if (rad >= ONE_PI)
	{
		rad -= ONE_PI;
		flag_ = -1;
	}

	return mx_sin(rad) * flag_;
}

/*******************************************************************************
* 函 数 名         : my_cos
* 函数功能		     : 计算cos
* 输    入         : 角度rad
* 输    出         : sin
*******************************************************************************/
float my_cos(double rad)
{
	s8 _flag = 1;
	rad += ONE_PI/2.0;

	if (rad >= ONE_PI)
	{
		_flag = -1;
		rad -= ONE_PI;
	}

	return my_sin(rad)*_flag;
}

/*******************************************************************************
* 函 数 名         : my_deadzone
* 函数功能		     : 死区限制
* 输    入         : 输入值x 中间值medi 死区值zoom
* 输    出         : sin
*******************************************************************************/
float my_deadzone(float x,float medi,float zoom)
{
	float t;
	if(x>medi)
	{
		t = x - zoom;
		if(t<medi)
		{
			t = medi;
		}
	}
	else
	{
		t = x + zoom;
		if(t>medi)
		{
			t = medi;
		}
	}
  return (t);
}

/*******************************************************************************
* 函 数 名         : inte_fix_filter
* 函数功能		     : 积分补偿滤波器
* 输    入         : 周期s 滤波数据结构体
* 输    出         : sin
*******************************************************************************/
void inte_fix_filter(float dT_s,_INTE_FIX_Filter_structure *data)
{
	float ei_lim_val;
	
	if(data->ei_limit>0)
	{		
		ei_lim_val = LIMIT(data->ei,-data->ei_limit,data->ei_limit);
	}
	else
	{
		ei_lim_val = data->ei;
	}	
	
	data->out = (data->in_est + ei_lim_val);
	
	data->e = data->fix_ki *(data->in_obs - data->out);

	data->ei += data->e *dT_s;


}

/*******************************************************************************
* 函 数 名         : diff_fix_filter
* 函数功能		     : 补偿积分滤波器
* 输    入         : 周期s 滤波数据结构体
* 输    出         : sin
*******************************************************************************/
void fix_inte_filter(float dT_s,_FIX_INTE_Filter_structure *data)
{
	
	data->out += (data->in_est_d + data->e ) *dT_s;
	
	data->e = data->fix_kp *(data->in_obs - data->out);

	if(data->e_limit>0)
	{		
		data->e = LIMIT(data->e,-data->e_limit,data->e_limit);
	}
	
	
}

