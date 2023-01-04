#ifndef __PAR_MANAGE_H__
#define __PAR_MANAGE_H__

//#include "io.h"
#include "stm32f4xx.h"

#define STORE_DELAY 500       //开始保存参数前的延时 方便数据传输校验完成

#define LOAD_MAX 64           //参数列表在eeprom里的大小

#define PAR_NUM 23            //参数个数

#define PAR_DESC_FRAME_LEN 50 //参数信息帧大小

#define PAR_NAME_LEN 20       //参数名字长度
 
#define PAR_INFO_LEN 20       //参数信息长度

//飞行参数结构体
typedef struct{
	s16 pit_offset; //imu pit rol水平校准偏置 单位度*100 0-9000
	s16 rol_offset;
	
	s16 ch1_direct;    //通道124 输出正反 1/0
	s16 ch2_direct;
	s16 ch4_direct;
	
	s16 ch1_offset;   //通道1234输出的偏置 直接参与pwm输出 -100-100
	s16 ch2_offset;
	s16 ch3_offset;
	s16 ch4_offset;
	
	s16 ratio_att_rol;     //pit rol 自稳等模式感度 50为1
	s16 ratio_att_pit; 
	
	s16 ratio_rotation_rol; //pit rol 运动模式感度 50为1
	s16 ratio_rotation_pit;
	
	s16 ctrl_mode;     //固定翼模式 0三角翼 1传统布局
	
	s16 rol_angle_max; //自稳模式下 最大倾斜角度 单位度 0-90
	s16 pit_angle_max;                                  
	
					  //除手动模式下 最大旋转速度 单位度/s
	s16 rol_angular_spd_max;
	s16 pit_angular_spd_max;
	
					  //自动巡航模式下 空速设定值 和 海拔高度设定值
	s16 target_air_speed;      //单位 m/s  0-127
	s16 target_autoft_altitude;//单位 m    0-127
	s16 auto_thr_max;          //最大油门  0-100
	s16 auto_thr_min;          //最小油门  0-100
	s16 auto_pit_max;          //pit限制值 0-90
	
} PAR_structure;

//参数信息结构体
typedef struct{
	s16 par_max;           //参数最大值
	s16 par_min;           //参数最小值
	u8 name[PAR_NAME_LEN]; //参数名字 字符串
	u8 info[PAR_INFO_LEN]; //参数信息 字符串
} PAR_INFO_structure;

//用于参数的保存
typedef union{
	PAR_structure par;
	u8 raw_data[LOAD_MAX];
} PAR_Store_union;

//用于参数的修改和读出
typedef union{
	PAR_structure par;
	s16 s16_unit_array[LOAD_MAX/2];
} PAR_MSG_union;

extern PAR_MSG_union fl_par;
extern PAR_Store_union storaged_par;
extern PAR_INFO_structure par_info_array[PAR_NUM];

void PAR_Load(void);
void PAR_Store(void);

void PAR_Change(u8 par_id, float par_val);
void PAR_Store_Task(u8 dT_ms);


#endif
