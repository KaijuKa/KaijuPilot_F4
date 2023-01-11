#include "pos_calcu.h"
#include "bmp280.h"
#include "delay.h"

POS_Structure pos_data = {
	.baro_height = 0,
	.baro_height_err = 0,
	.fusion_height = 0
};

/*******************************************************************************
* 函 数 名         : POS_Init
* 函数功能		   : 初始化POS计算
* 输    入         : 无
* 输    出         : 无
*******************************************************************************/
void POS_Init(void)
{
	RAW_Height_Calibration();
}

/*******************************************************************************
* 函 数 名         : RAW_Height_Calibration
* 函数功能		   : 计算原生高度初始值
* 输    入         : 无
* 输    出         : 无
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
* 函 数 名         : POS_Update
* 函数功能		   : 进行位置相关信息的更新
* 输    入         : 周期ms
* 输    出         : 无
*******************************************************************************/
void POS_Update(u8 dT_ms)
{
	RAW_Height_Update(dT_ms);
	Height_Fusion(dT_ms);
}

/*******************************************************************************
* 函 数 名         : RAW_Height_Update
* 函数功能		   : 获取原生高度信息
* 输    入         : 周期ms
* 输    出         : 无
*******************************************************************************/
void RAW_Height_Update(u8 dT_ms)
{
	float bmp_temp;
	float bmp_press;
	float tmp, raw_height;
	
	//等待测量和更新完毕
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
* 函 数 名         : Height_Fusion
* 函数功能		   : 进行高度数据融合
* 输    入         : 周期ms
* 输    出         : 无
*******************************************************************************/
void Height_Fusion(u8 dT_ms)
{
	pos_data.fusion_height = pos_data.baro_height;
}


