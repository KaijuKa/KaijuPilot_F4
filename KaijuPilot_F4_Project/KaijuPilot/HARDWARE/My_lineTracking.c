#include "My_lineTracking.h"
#include "swimming_ctrl.h"

//需要调用引用的外部变量：
#define DEAD_TIME               1000               //点追踪丢失时间
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))	
#define MY_PPPIII   3.14159f

//参数设定：
#define CT_KP                  (1.35f)  //比例项
#define CT_KD                  (0.16f)  //微分项
#define CT_KI                  (0.80f)  //积分项
//
#define CMPPIXEL_X     0.01f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。//目前粗略标定
#define CMPPIXEL_Y     0.01f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。
#define TLH_TIME       1000   //判断目标丢失的保持时间。
#define TRACK_VEL      25    //追踪速度
#define TRACK_MODE     1      //0 无头模式 1 有头模式即头会转

_openmv_line_tracking_st my_opmv;
opmv_lt2 my_opmv_lt2;         //点追踪结构体
u16 target_loss_time_lt2;   //点追踪中目标丢失时间
s16 pos_x_err_ct;              //跟踪时x方向偏移量
s16 pos_y_err_ct;              //跟踪时y方向偏移量
s16 track_vel = TRACK_VEL;


/**********************************************************************************************************
*函 数 名: opmv_ct_Decoupling
*功能说明: opmv线追踪2 追拟合后线的中点的解耦合
*参    数: 无
*返 回 值: 无
*日    期：2021.7.25
**********************************************************************************************************/
void opmv_ct_Decoupling(void)
{                                            //获取机体速度
	//opmv坐标转机体坐标 +上偏移量
	if(my_opmv.sta )
	{
		my_opmv_lt2.post_xy[0] =  my_opmv.pos_y-pos_x_err_ct;
		my_opmv_lt2.post_xy[1] = -my_opmv.pos_x-pos_y_err_ct;
	}
	else
	{
		my_opmv_lt2.post_xy[0] =  0;
		my_opmv_lt2.post_xy[1] =  0;
	}
	
	                                           //数据耦合二次
	my_opmv_lt2.post_final[0][0] = 0.8f*my_opmv_lt2.post_final[0][0]+0.2f*(my_opmv_lt2.post_xy[0]);
	my_opmv_lt2.post_final[0][1] = 0.8f*my_opmv_lt2.post_final[0][1]+0.2f*(my_opmv_lt2.post_xy[1]);
	
	my_opmv_lt2.post_final[1][0] = 0.8f*my_opmv_lt2.post_final[1][0]+0.2f*(my_opmv_lt2.post_final[0][0]);
	my_opmv_lt2.post_final[1][1] = 0.8f*my_opmv_lt2.post_final[1][1]+0.2f*(my_opmv_lt2.post_final[0][1]);
}

/**********************************************************************************************************
*函 数 名: opmv_ct_Calculating
*功能说明: opmv线追踪2 追拟合后线的中点的计算
*参    数: 周期(ms)
*返 回 值: 无
*日    期：2020.6.26  
**********************************************************************************************************/
void opmv_ct_Calculating(u8 dT_ms)
{                                            //相对高度赋值
	static float relative_height_cm_valid;
	relative_height_cm_valid = 10;
	
	                                           //存储上一次的位置信息
	my_opmv_lt2.post_reality_cm[1][0] = my_opmv_lt2.post_reality_cm[0][0];
	my_opmv_lt2.post_reality_cm[1][1] = my_opmv_lt2.post_reality_cm[0][1];
	                                           //计算真实位置信息
	my_opmv_lt2.post_reality_cm[0][0] = my_opmv_lt2.post_final[1][0]* CMPPIXEL_X *relative_height_cm_valid;
	my_opmv_lt2.post_reality_cm[0][1] = my_opmv_lt2.post_final[1][1]* CMPPIXEL_Y *relative_height_cm_valid;
	                                           //计算积分项
	//my_opmv_lt2.post_err_i += my_opmv_lt2.post_reality_cm[0][1]*CT_KI/(1000/dT_ms);
	                                           //位置信息微分
	my_opmv_lt2.post_err_diff[0] = 0.8f*my_opmv_lt2.post_err_diff[0]+0.2f*(my_opmv_lt2.post_reality_cm[0][0]-my_opmv_lt2.post_reality_cm[1][0])*1000/(dT_ms);
	my_opmv_lt2.post_err_diff[1] = 0.8f*my_opmv_lt2.post_err_diff[1]+0.2f*(my_opmv_lt2.post_reality_cm[0][1]-my_opmv_lt2.post_reality_cm[1][1])*1000/(dT_ms);
}

float angle = 0;
/**********************************************************************************************************
*函 数 名: opmv_lt2_Ctrl
*功能说明: opmv线追踪2的应用程序
*参    数: 标志(1：不再运动 0：运动)
*返 回 值: 无
*日    期：2020.6.26  
**********************************************************************************************************/
void opmv_lt2_Ctrl(u8 uflag)
{                                            //计算PDF结果值
																						 //根据模式不同换算角度
	angle = 0.7f*angle + 0.3f*( TRACK_MODE ? (my_opmv.angle > 90 ? (-180+my_opmv.angle)*0.8f :my_opmv.angle*0.8f) \
	: (my_opmv.angle < 90 ? 90 - my_opmv.angle : 270-my_opmv.angle));
	
	angle = MY_PPPIII*angle/180.0f;               //切换成弧度
	
	my_opmv_lt2.output_final_PDF[0]\
	= CT_KP *my_opmv_lt2.post_reality_cm[0][0]\
	+ CT_KD *my_opmv_lt2.post_err_diff[0]\
	+ track_vel;//加上预设速度的分量
	
	my_opmv_lt2.output_final_PDF[1]\
	= CT_KP *my_opmv_lt2.post_reality_cm[0][1]\
	+ CT_KD *my_opmv_lt2.post_err_diff[1]\
	+	my_opmv_lt2.post_err_i\
	+ (!TRACK_MODE ? -my_cos(angle)*track_vel : 0);//加上预设速度的分量

	                                              //限幅
	my_opmv_lt2.output_final_PDF[0] = LIMIT(my_opmv_lt2.output_final_PDF[0],-TRACK_VEL,TRACK_VEL);
	my_opmv_lt2.output_final_PDF[1] = LIMIT(my_opmv_lt2.output_final_PDF[1],-TRACK_VEL,TRACK_VEL);
	
	if(uflag == 0)                             //应用程控函数控制速度 当目标丢失时 按原来的速度继续行进 当丢失超时时 停止运动
	{
//		Program_XYspd_set(my_opmv_lt2.output_final_PDF[0],my_opmv_lt2.output_final_PDF[1]);
//		Program_YAWspd_set(TRACK_MODE ? 35.0f*angle : 0);
		Program_XYspd_set(my_opmv_lt2.output_final_PDF[0],0);
		Program_YAWspd_set(135.0f*angle + my_opmv_lt2.output_final_PDF[1]*2.7f);
	}
	else
	{
		Program_XYspd_set(0,0);
		Program_YAWspd_set(0);
	}
}

/**********************************************************************************************************
*函 数 名: opmv_lt2_ctrl_task
*功能说明: opmv线追踪2的控制函数
*参    数: 周期(ms)
*返 回 值: 无
*日    期：2020.6.26  
**********************************************************************************************************/
void opmv_lt2_ctrl_task(u8 dT_ms)
{                                           
	                                           //判断是否可以进行点追踪
	if(!my_opmv.offline)
	{
		if(!my_opmv.sta)
		{	                                       //丢失目标
			if(target_loss_time_lt2 >= DEAD_TIME)
				target_loss_time_lt2 += dT_ms;
			else
				my_opmv_lt2.target_loss_uflag = 1;
			                                       //目标丢失 实际距离低通复位到0
			LPF_1_(0.2f,dT_ms/1000.0f,0,my_opmv_lt2.post_reality_cm[0][0]);
			LPF_1_(0.2f,dT_ms/1000.0f,0,my_opmv_lt2.post_reality_cm[0][1]);
			LPF_1_(0.2f,dT_ms/1000.0f,0,my_opmv_lt2.post_err_i);
		}	
		else
		{                                        //目标没有丢失
			target_loss_time_lt2 = 0;
			my_opmv_lt2.target_loss_uflag = 0;
			opmv_ct_Decoupling();
			opmv_ct_Calculating(dT_ms);
		}      
																						 //执行opmv ct应用程序
		opmv_lt2_Ctrl(my_opmv_lt2.target_loss_uflag);
	}
	else
	{
		my_opmv_lt2.post_err_i = 0;
		Program_XYspd_set(0,0);		
		Program_YAWspd_set(0);
	}
}

