#include "arg_manage.h"

ARG_structure flight_arg = {
	.pit_offset = 0,
	.rol_offset = 0,
	
	.ch1_direct = 1,
	.ch2_direct = 1,
	.ch4_direct = 0,
	
	.ch1_offset = 0,
	.ch2_offset = 0,
	.ch3_offset = 0,
	.ch4_offset = 0,
	
	.ratio_att_rol = 21,
	.ratio_att_pit = 50,
	
	.ratio_rotation_rol = 21,
	.ratio_rotation_pit = 50,
	
	.ctrl_mode = 1,
	
	.rol_angle_max = 65,
	.pit_angle_max = 50,
	
	.rol_angular_spd_max = 200,
	.pit_angular_spd_max = 150
};
