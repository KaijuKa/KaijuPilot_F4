#include "bmp280.h"
#include "spi.h"
#include "delay.h"
 
BMP280_FIX_PAR_Structure bmp280_fix_par_data;

/*******************************************************************************
* �� �� ��         : bmp280_enable
* ��������		   : bmp280Ƭѡ
* ��    ��         : 1 Ƭѡ 0 ��Ƭѡ
* ��    ��         : ��
*******************************************************************************/
static void bmp280_enable(u8 ena)
{
	if(ena)
		GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	else
		GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

/*******************************************************************************
* �� �� ��         : bmp280_readbuf
* ��������		   : ��bmp280�д�ָ���Ĵ�����ʼ����length���ȵ�����
* ��    ��         : reg��ʼ�Ĵ�����ַ length���ݳ��� data���ݴ洢��ַ
* ��    ��         : ��
*******************************************************************************/
static void bmp280_readbuf(u8 reg, u8 length, u8 *data)
{
	bmp280_enable(1);
	DRV_SPI2_Read_Write_Byte(reg | 0x80);
	DRV_SPI2_Receive(data,length);
	bmp280_enable(0);
}

/*******************************************************************************
* �� �� ��         : bmp280_writebyte
* ��������		   : ��bmp280�д�reg��ʼд��һ���ֽ�����
* ��    ��         : reg��ʼ�Ĵ�����ַ data�ֽ�����
* ��    ��         : ������һ���ֽ�����
*******************************************************************************/
static u8 bmp280_writebyte(u8 reg, u8 data)
{
	u8 status;
	
	bmp280_enable(1);
	status = DRV_SPI2_Read_Write_Byte(reg & 0x7F);
	DRV_SPI2_Read_Write_Byte(data);
	bmp280_enable(0);
	return status;
}

/*******************************************************************************
* �� �� ��         : DRV_Bmp280_Init
* ��������		   : ����bmp280
* ��    ��         : ��
* ��    ��         : 0�ɹ� 1ʧ��
*******************************************************************************/
u8 BMP_ID;
u8 DRV_Bmp280_Init(void)
{
	u8 Lsb,Msb;
	//����ID
	bmp280_readbuf(BMP280_CHIPID_REG, 1, &BMP_ID);

	//�������ֵ
	//�¶ȴ������Ľ���ֵ
	bmp280_readbuf(BMP280_DIG_T1_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_T1_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.T1 = (((u16)Msb)<<8) + Lsb;
	bmp280_readbuf(BMP280_DIG_T2_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_T2_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.T2 = (((u16)Msb)<<8) + Lsb;		
	bmp280_readbuf(BMP280_DIG_T3_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_T3_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.T3 = (((u16)Msb)<<8) + Lsb;		
	
	//����ѹ�������Ľ���ֵ
	bmp280_readbuf(BMP280_DIG_P1_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P1_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P1 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P2_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P2_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P2 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P3_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P3_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P3 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P4_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P4_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P4 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P5_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P5_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P5 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P6_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P6_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P6 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P7_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P7_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P7 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P8_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P8_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P8 = (((u16)Msb)<<8) + Lsb;	
	bmp280_readbuf(BMP280_DIG_P9_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_DIG_P9_MSB_REG, 1, &Msb);
	bmp280_fix_par_data.P9 = (((u16)Msb)<<8) + Lsb;	
	delay_ms(10);
	
	//����λ�Ĵ���д�����ֵ
	bmp280_writebyte(BMP280_RESET_REG, BMP280_RESET_VAL);
	delay_ms(10);
	
	//��λ�Ƽ�����
	//010b x2�¶ȹ�����; 101b x16ѹǿ������; 11b normal mode;
	bmp280_writebyte(BMP280_CTRLMEAS_REG, BMP280_CTRLMEAS_VAL);
	delay_ms(10);
	
	//000b 0.5ms�����ȶ����; 100b x16�˲���; 00b �ر�3��spi,����4��spi;
	bmp280_writebyte(BMP280_CONFIG_REG, BMP280_CONFIG_VAL);
	delay_ms(10);
	
	if(BMP280_ID == BMP_ID)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

/**************************������ֵת����ֵ*************************************/
BMP280_S32_t t_fine;			//���ڼ��㲹��
#define USE_FIXED_POINT_COMPENSATE
//���ø��㲹��
#ifdef USE_FIXED_POINT_COMPENSATE
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of ��5123�� equals 51.23 DegC. 
// t_fine carries fine temperature as global value
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) * 
	((BMP280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of ��24674867�� represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)p;
}


/***********************************CUT*************************************/
#else
/**************************������ֵת����ֵ*************************************/
// Returns temperature in DegC, double precision. Output value of ��51.23�� equals 51.23 DegC.
// t_fine carries fine temperature as global value
double bmp280_compensate_T_double(BMP280_S32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (BMP280_S32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of ��96386.2�� equals 96386.2 Pa = 963.862 hPa
double bmp280_compensate_P_double(BMP280_S32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}
#endif

/*******************************************************************************
* �� �� ��         : DRV_Bmp280_GetPressure
* ��������		   : ��ȡbmp280ѹǿֵ
* ��    ��         : ��
* ��    ��         : ����ѹǿֵ Pa
*******************************************************************************/
float DRV_Bmp280_GetPressure(void)
{
	u8 XLsb,Lsb, Msb;
	long signed Bit32;
	float pressure;
	
	bmp280_readbuf(BMP280_PRESSURE_XLSB_REG, 1, &XLsb);
	bmp280_readbuf(BMP280_PRESSURE_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_PRESSURE_MSB_REG, 1, &Msb);
	
	//�Ĵ�����ֵ,���һ��������
	Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);
	pressure = bmp280_compensate_P_int64(Bit32) / 256.0f;
	return pressure;
}

/*******************************************************************************
* �� �� ��         : DRV_Bmp280_GetTemperature
* ��������		   : ��ȡbmp280�¶�ֵ
* ��    ��         : ��
* ��    ��         : �¶�ֵ ���϶�
*******************************************************************************/
float DRV_Bmp280_GetTemperature(void)
{
	uint8_t XLsb,Lsb, Msb;
	long signed Bit32;
	float temperature;
	
	bmp280_readbuf(BMP280_TEMPERATURE_XLSB_REG, 1, &XLsb);
	bmp280_readbuf(BMP280_TEMPERATURE_LSB_REG, 1, &Lsb);
	bmp280_readbuf(BMP280_TEMPERATURE_MSB_REG, 1, &Msb);
	
	//�Ĵ�����ֵ,���һ��������
	Bit32 = ((long)(Msb << 12))|((long)(Lsb << 4))|(XLsb>>4);
	temperature = bmp280_compensate_T_int32(Bit32) / 100.0f;
	return temperature;
}

/*******************************************************************************
* �� �� ��         : DRV_Bmp280_DataEnable
* ��������		   : �鿴�����Ƿ���Զ�ȡ
* ��    ��         : ��
* ��    ��         : 0 ����ȡ���� 1 ������ȡ����
*******************************************************************************/
u8 DRV_Bmp280_DataEnable(void)
{
	u8 flag;
	bmp280_readbuf(BMP280_STATUS_REG, 1, &flag);
	
	flag &= (BMP280_MEASURING | BMP280_IN_UPDATE);
	
	if(0 == flag)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

