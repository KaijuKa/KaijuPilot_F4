#include "pos_calcu.h"
#include "bmp280.h"
#include "delay.h"
#include "kaiju_math.h"
#include "imu.h"

#include "math.h"

#define N_TIMES 5

POS_Structure pos_data = {
	.baro_height = 0,
	.baro_height_offset = 0,
	.fusion_height = 0
};

//���ֲ�����ؽṹ��
_INTE_FIX_Filter_structure wcz_acc_fus;
//����������ؽṹ��
_FIX_INTE_Filter_structure wcz_spe_fus,wcz_hei_fus;

//�߶��ں���Ҫ�ĸ��ֹ۲�����
s32 ref_height_old,ref_speed_old;
s32 wcz_ref_height,wcz_ref_speed,wcz_ref_acc;
s32 wcz_acc;

/*******************************************************************************
* �� �� ��         : POS_Init
* ��������		   : ��ʼ��POS����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void POS_Init(void)
{
	;
}

/*******************************************************************************
* �� �� ��         : RAW_Height_Calibration
* ��������		   : ����ԭ���߶ȳ�ʼֵ
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void RAW_Height_Calibration(void)
{
	pos_data.baro_height_offset = pos_data.baro_height;
}

/*******************************************************************************
* �� �� ��         : POS_Update
* ��������		   : ����λ�������Ϣ�ĸ���
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void POS_Update(u8 dT_ms)
{
	RAW_Height_Update(dT_ms);
	Height_Fusion(dT_ms);
}

/*******************************************************************************
* �� �� ��         : RAW_Height_Update
* ��������		   : ��ȡԭ���߶���Ϣ
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void RAW_Height_Update(u8 dT_ms)
{
	float bmp_press;
	float tmp;
	static float raw_height = 0;
	
	//�ȴ������͸������
	if(DRV_Bmp280_DataEnable() == 0)
	{
		bmp_press = DRV_Bmp280_GetPressure();
		
		tmp = ( 101325 - bmp_press ) / 1000.0f;
		raw_height = 0.82f * tmp * tmp * tmp + 9.0f * ( 101325 - bmp_press );
		raw_height -= pos_data.baro_height_offset;
	}
	
	//�򵥵�ͨ�˲�
	pos_data.baro_height += 0.007f*(raw_height - pos_data.baro_height);
}

/*******************************************************************************
* �� �� ��         : Height_Fusion
* ��������		   : ���и߶������ں�
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void Height_Fusion(u8 dT_ms)
{
	static u8 cyc_xn;
	float hz,ntimes_hz;	
	hz = safe_div(1000,dT_ms,0);
	ntimes_hz = hz/N_TIMES;
	
	wcz_ref_height = pos_data.baro_height;
	//��ͨ�˲�
	wcz_acc += 0.07f * (imu_data.w_acc[2] - wcz_acc);
	
	//ÿ200ms���¹۲�����
	cyc_xn ++;
	cyc_xn %= N_TIMES;
	
	if(cyc_xn == 0)
	{
		wcz_ref_speed = (wcz_ref_height - ref_height_old) *ntimes_hz;
		
		wcz_ref_acc = (wcz_ref_speed - ref_speed_old) *ntimes_hz;
		
		ref_height_old = wcz_ref_height;	
		ref_speed_old = wcz_ref_speed;
	}
	
	wcz_acc_fus.fix_ki = 0.1f;
	wcz_acc_fus.in_est = wcz_acc;
	wcz_acc_fus.in_obs = wcz_ref_acc;
	wcz_acc_fus.ei_limit = 100;
	inte_fix_filter(dT_ms*1e-3f,&wcz_acc_fus);

	
	wcz_spe_fus.fix_kp = 0.6f;
	wcz_spe_fus.in_est_d = wcz_acc_fus.out;
	wcz_spe_fus.in_obs = wcz_ref_speed;
	wcz_spe_fus.e_limit = 100;
	fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);
	
	
	wcz_hei_fus.fix_kp = 0.25f;
	wcz_hei_fus.in_est_d = wcz_spe_fus.out;
	wcz_hei_fus.in_obs = pos_data.baro_height;
	//wcz_hei_fus.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&wcz_hei_fus);
	
	pos_data.fusion_height = wcz_hei_fus.out;
}

