#include "pos_calcu.h"
#include "bmp280.h"
#include "delay.h"

POS_Structure pos_data = {
	.baro_height = 0,
	.baro_height_err = 0,
	.fusion_height = 0
};

/*******************************************************************************
* �� �� ��         : POS_Init
* ��������		   : ��ʼ��POS����
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void POS_Init(void)
{
	RAW_Height_Calibration();
}

/*******************************************************************************
* �� �� ��         : RAW_Height_Calibration
* ��������		   : ����ԭ���߶ȳ�ʼֵ
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void RAW_Height_Calibration(void)
{
	float bmp_temp;
	float bmp_press;
	float tmp, raw_height;
	u8 i;
	
	for(i = 0; i < 100; i++)
	{
		while(DRV_Bmp280_DataEnable() != 0);
		
		bmp_temp = DRV_Bmp280_GetTemperature();
		bmp_press = DRV_Bmp280_GetPressure();

		tmp = ( 101325 - bmp_press ) / 1000.0f;
		raw_height = 0.82f * tmp * tmp * tmp + 9.0f * ( 101325 - bmp_press );
		
		pos_data.baro_height_err += raw_height;
		
		delay_ms(1);
	}
	pos_data.baro_height_err /= 100;
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
	float bmp_temp;
	float bmp_press;
	float tmp, raw_height;
	
	//�ȴ������͸������
	if(DRV_Bmp280_DataEnable() == 1)
	{
		return;
	}
	
	bmp_temp = DRV_Bmp280_GetTemperature();
	bmp_press = DRV_Bmp280_GetPressure();
		
	tmp = ( 101325 - bmp_press ) / 1000.0f;
	raw_height = 0.82f * tmp * tmp * tmp + 9.0f * ( 101325 - bmp_press );
	pos_data.baro_height = raw_height - pos_data.baro_height_err;
}

/*******************************************************************************
* �� �� ��         : Height_Fusion
* ��������		   : ���и߶������ں�
* ��    ��         : ����ms
* ��    ��         : ��
*******************************************************************************/
void Height_Fusion(u8 dT_ms)
{
	pos_data.fusion_height = pos_data.baro_height;
}


