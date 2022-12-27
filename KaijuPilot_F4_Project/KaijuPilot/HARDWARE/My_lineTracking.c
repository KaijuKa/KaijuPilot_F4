#include "My_lineTracking.h"
#include "swimming_ctrl.h"

//��Ҫ�������õ��ⲿ������
#define DEAD_TIME               1000               //��׷�ٶ�ʧʱ��
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))	
#define MY_PPPIII   3.14159f

//�����趨��
#define CT_KP                  (1.35f)  //������
#define CT_KD                  (0.16f)  //΢����
#define CT_KI                  (0.80f)  //������
//
#define CMPPIXEL_X     0.01f     //ÿ���ض�Ӧ�ĵ�����룬�뽹��͸߶��йأ���Ҫ���Ա궨��//Ŀǰ���Ա궨
#define CMPPIXEL_Y     0.01f     //ÿ���ض�Ӧ�ĵ�����룬�뽹��͸߶��йأ���Ҫ���Ա궨��
#define TLH_TIME       1000   //�ж�Ŀ�궪ʧ�ı���ʱ�䡣
#define TRACK_VEL      25    //׷���ٶ�
#define TRACK_MODE     1      //0 ��ͷģʽ 1 ��ͷģʽ��ͷ��ת

_openmv_line_tracking_st my_opmv;
opmv_lt2 my_opmv_lt2;         //��׷�ٽṹ��
u16 target_loss_time_lt2;   //��׷����Ŀ�궪ʧʱ��
s16 pos_x_err_ct;              //����ʱx����ƫ����
s16 pos_y_err_ct;              //����ʱy����ƫ����
s16 track_vel = TRACK_VEL;


/**********************************************************************************************************
*�� �� ��: opmv_ct_Decoupling
*����˵��: opmv��׷��2 ׷��Ϻ��ߵ��е�Ľ����
*��    ��: ��
*�� �� ֵ: ��
*��    �ڣ�2021.7.25
**********************************************************************************************************/
void opmv_ct_Decoupling(void)
{                                            //��ȡ�����ٶ�
	//opmv����ת�������� +��ƫ����
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
	
	                                           //������϶���
	my_opmv_lt2.post_final[0][0] = 0.8f*my_opmv_lt2.post_final[0][0]+0.2f*(my_opmv_lt2.post_xy[0]);
	my_opmv_lt2.post_final[0][1] = 0.8f*my_opmv_lt2.post_final[0][1]+0.2f*(my_opmv_lt2.post_xy[1]);
	
	my_opmv_lt2.post_final[1][0] = 0.8f*my_opmv_lt2.post_final[1][0]+0.2f*(my_opmv_lt2.post_final[0][0]);
	my_opmv_lt2.post_final[1][1] = 0.8f*my_opmv_lt2.post_final[1][1]+0.2f*(my_opmv_lt2.post_final[0][1]);
}

/**********************************************************************************************************
*�� �� ��: opmv_ct_Calculating
*����˵��: opmv��׷��2 ׷��Ϻ��ߵ��е�ļ���
*��    ��: ����(ms)
*�� �� ֵ: ��
*��    �ڣ�2020.6.26  
**********************************************************************************************************/
void opmv_ct_Calculating(u8 dT_ms)
{                                            //��Ը߶ȸ�ֵ
	static float relative_height_cm_valid;
	relative_height_cm_valid = 10;
	
	                                           //�洢��һ�ε�λ����Ϣ
	my_opmv_lt2.post_reality_cm[1][0] = my_opmv_lt2.post_reality_cm[0][0];
	my_opmv_lt2.post_reality_cm[1][1] = my_opmv_lt2.post_reality_cm[0][1];
	                                           //������ʵλ����Ϣ
	my_opmv_lt2.post_reality_cm[0][0] = my_opmv_lt2.post_final[1][0]* CMPPIXEL_X *relative_height_cm_valid;
	my_opmv_lt2.post_reality_cm[0][1] = my_opmv_lt2.post_final[1][1]* CMPPIXEL_Y *relative_height_cm_valid;
	                                           //���������
	//my_opmv_lt2.post_err_i += my_opmv_lt2.post_reality_cm[0][1]*CT_KI/(1000/dT_ms);
	                                           //λ����Ϣ΢��
	my_opmv_lt2.post_err_diff[0] = 0.8f*my_opmv_lt2.post_err_diff[0]+0.2f*(my_opmv_lt2.post_reality_cm[0][0]-my_opmv_lt2.post_reality_cm[1][0])*1000/(dT_ms);
	my_opmv_lt2.post_err_diff[1] = 0.8f*my_opmv_lt2.post_err_diff[1]+0.2f*(my_opmv_lt2.post_reality_cm[0][1]-my_opmv_lt2.post_reality_cm[1][1])*1000/(dT_ms);
}

float angle = 0;
/**********************************************************************************************************
*�� �� ��: opmv_lt2_Ctrl
*����˵��: opmv��׷��2��Ӧ�ó���
*��    ��: ��־(1�������˶� 0���˶�)
*�� �� ֵ: ��
*��    �ڣ�2020.6.26  
**********************************************************************************************************/
void opmv_lt2_Ctrl(u8 uflag)
{                                            //����PDF���ֵ
																						 //����ģʽ��ͬ����Ƕ�
	angle = 0.7f*angle + 0.3f*( TRACK_MODE ? (my_opmv.angle > 90 ? (-180+my_opmv.angle)*0.8f :my_opmv.angle*0.8f) \
	: (my_opmv.angle < 90 ? 90 - my_opmv.angle : 270-my_opmv.angle));
	
	angle = MY_PPPIII*angle/180.0f;               //�л��ɻ���
	
	my_opmv_lt2.output_final_PDF[0]\
	= CT_KP *my_opmv_lt2.post_reality_cm[0][0]\
	+ CT_KD *my_opmv_lt2.post_err_diff[0]\
	+ track_vel;//����Ԥ���ٶȵķ���
	
	my_opmv_lt2.output_final_PDF[1]\
	= CT_KP *my_opmv_lt2.post_reality_cm[0][1]\
	+ CT_KD *my_opmv_lt2.post_err_diff[1]\
	+	my_opmv_lt2.post_err_i\
	+ (!TRACK_MODE ? -my_cos(angle)*track_vel : 0);//����Ԥ���ٶȵķ���

	                                              //�޷�
	my_opmv_lt2.output_final_PDF[0] = LIMIT(my_opmv_lt2.output_final_PDF[0],-TRACK_VEL,TRACK_VEL);
	my_opmv_lt2.output_final_PDF[1] = LIMIT(my_opmv_lt2.output_final_PDF[1],-TRACK_VEL,TRACK_VEL);
	
	if(uflag == 0)                             //Ӧ�ó̿غ��������ٶ� ��Ŀ�궪ʧʱ ��ԭ�����ٶȼ����н� ����ʧ��ʱʱ ֹͣ�˶�
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
*�� �� ��: opmv_lt2_ctrl_task
*����˵��: opmv��׷��2�Ŀ��ƺ���
*��    ��: ����(ms)
*�� �� ֵ: ��
*��    �ڣ�2020.6.26  
**********************************************************************************************************/
void opmv_lt2_ctrl_task(u8 dT_ms)
{                                           
	                                           //�ж��Ƿ���Խ��е�׷��
	if(!my_opmv.offline)
	{
		if(!my_opmv.sta)
		{	                                       //��ʧĿ��
			if(target_loss_time_lt2 >= DEAD_TIME)
				target_loss_time_lt2 += dT_ms;
			else
				my_opmv_lt2.target_loss_uflag = 1;
			                                       //Ŀ�궪ʧ ʵ�ʾ����ͨ��λ��0
			LPF_1_(0.2f,dT_ms/1000.0f,0,my_opmv_lt2.post_reality_cm[0][0]);
			LPF_1_(0.2f,dT_ms/1000.0f,0,my_opmv_lt2.post_reality_cm[0][1]);
			LPF_1_(0.2f,dT_ms/1000.0f,0,my_opmv_lt2.post_err_i);
		}	
		else
		{                                        //Ŀ��û�ж�ʧ
			target_loss_time_lt2 = 0;
			my_opmv_lt2.target_loss_uflag = 0;
			opmv_ct_Decoupling();
			opmv_ct_Calculating(dT_ms);
		}      
																						 //ִ��opmv ctӦ�ó���
		opmv_lt2_Ctrl(my_opmv_lt2.target_loss_uflag);
	}
	else
	{
		my_opmv_lt2.post_err_i = 0;
		Program_XYspd_set(0,0);		
		Program_YAWspd_set(0);
	}
}

