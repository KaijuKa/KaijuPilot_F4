#ifndef __BMP280_H__
#define __BMP280_H__

#include "stm32f4xx.h"

#define BMP280_ID						    0x58		//从设备地址
#define BMP280_RESET_VAL  					0xB6		//复位寄存器写入值
#define BMP280_CTRLMEAS_VAL                 0x57        //CTRLMEAS寄存器配置值
#define BMP280_CONFIG_VAL                   0x10        //CONFIG寄存器配置值

#define BMP280_CHIPID_REG                    0xD0  /*Chip ID Register */
#define BMP280_RESET_REG                     0xE0  /*Softreset Register */
#define BMP280_STATUS_REG                    0xF3  /*Status Register */
#define BMP280_CTRLMEAS_REG                  0xF4  /*Ctrl Measure Register */
#define BMP280_CONFIG_REG                    0xF5  /*Configuration Register */
#define BMP280_PRESSURE_MSB_REG              0xF7  /*Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              0xF8  /*Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             0xF9  /*Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           0xFA  /*Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           0xFB  /*Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          0xFC  /*Temperature XLSB Reg */
//状态寄存器转换标志
#define	BMP280_MEASURING					0x08
#define	BMP280_IN_UPDATE					0x01

#define BMP280_DIG_T1_LSB_REG                0x88  /*calibration parameters */
#define BMP280_DIG_T1_MSB_REG                0x89
#define BMP280_DIG_T2_LSB_REG                0x8A
#define BMP280_DIG_T2_MSB_REG                0x8B
#define BMP280_DIG_T3_LSB_REG                0x8C
#define BMP280_DIG_T3_MSB_REG                0x8D
#define BMP280_DIG_P1_LSB_REG                0x8E
#define BMP280_DIG_P1_MSB_REG                0x8F
#define BMP280_DIG_P2_LSB_REG                0x90
#define BMP280_DIG_P2_MSB_REG                0x91
#define BMP280_DIG_P3_LSB_REG                0x92
#define BMP280_DIG_P3_MSB_REG                0x93
#define BMP280_DIG_P4_LSB_REG                0x94
#define BMP280_DIG_P4_MSB_REG                0x95
#define BMP280_DIG_P5_LSB_REG                0x96
#define BMP280_DIG_P5_MSB_REG                0x97
#define BMP280_DIG_P6_LSB_REG                0x98
#define BMP280_DIG_P6_MSB_REG                0x99
#define BMP280_DIG_P7_LSB_REG                0x9A
#define BMP280_DIG_P7_MSB_REG                0x9B
#define BMP280_DIG_P8_LSB_REG                0x9C
#define BMP280_DIG_P8_MSB_REG                0x9D
#define BMP280_DIG_P9_LSB_REG                0x9E
#define BMP280_DIG_P9_MSB_REG                0x9F

/*******************************下面是用来计算补偿值相关**********************************/
typedef			long signed int				BMP280_S32_t;	//有符号 64位！
typedef			long unsigned int			BMP280_U32_t;	//无符号 32位！
typedef			long long signed int		BMP280_S64_t;

#define	dig_T1			bmp280_fix_par_data.T1	
#define	dig_T2			bmp280_fix_par_data.T2	
#define	dig_T3			bmp280_fix_par_data.T3	

#define	dig_P1			bmp280_fix_par_data.P1
#define	dig_P2			bmp280_fix_par_data.P2
#define	dig_P3			bmp280_fix_par_data.P3
#define	dig_P4			bmp280_fix_par_data.P4
#define	dig_P5			bmp280_fix_par_data.P5
#define	dig_P6			bmp280_fix_par_data.P6
#define	dig_P7			bmp280_fix_par_data.P7
#define	dig_P8			bmp280_fix_par_data.P8
#define	dig_P9			bmp280_fix_par_data.P9
/************************************************CUT****************************************/


typedef struct  
{
	/* T1~P9 为补偿系数 */
	uint16_t T1;
	int16_t	T2;
	int16_t	T3;
	uint16_t P1;
	int16_t	P2;
	int16_t	P3;
	int16_t	P4;
	int16_t	P5;
	int16_t	P6;
	int16_t	P7;
	int16_t	P8;
	int16_t	P9;
} BMP280_FIX_PAR_Structure;

static void bmp280_enable(u8 ena);
static void bmp280_readbuf(u8 reg, u8 length, u8 *data);
static u8 bmp280_writebyte(u8 reg, u8 data);
u8 DRV_Bmp280_Init(void);
float DRV_Bmp280_GetPressure(void);
float DRV_Bmp280_GetTemperature(void);
u8 DRV_Bmp280_DataEnable(void);


#endif
