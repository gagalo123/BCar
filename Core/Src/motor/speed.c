#include"run_main.h"
#include"speed.h"
#include"stm32f1xx_hal.h"


SPEED_MEASURE_STRUCT vl, vr;

SPEED_STRUCT sp;
PID_SPEED_STRUCT PID_speed;
void SpeedControl_Init(PID_SPEED_STRUCT *pid, float target, double kp, double ki) {
//    printf("PID_SPEED_Init...\n");
    pid->target_val = target;
    pid->err =
    pid->inter =
    pid->pwm = 0;
		pid->err_last= 0 ;

    pid->kp = kp;
    pid->ki = ki;
}
extern short moving;
double his_inter_max = 0; // 9.¶à
double his_err_max = 0;
int SpeedControl_update(PID_SPEED_STRUCT *pid, double NowSpeed) {
//	pid->err = pid->target_val - NowSpeed;
//	pid->pwm +=pid->kp *(pid->err - pid->err_last)+
//				pid->ki * pid->err;
//	pid->err_last = pid->err;
//	const double err_max = 10000;
//	const double inter_max = 7000;
	pid->err = pid->target_val - NowSpeed;
	pid->err = pid->err * 0.3 + pid->err_last * 0.7;
	pid->inter += pid->err;
	pid->err_last = pid->err;
	//pid->inter -= moving*0.25;
	
//	pid->inter += moving;
//	if(moving)
//	{
//		if(pid->err>err_max) pid->err = err_max;
//		if(pid->err<-err_max) pid->err = -err_max;
//	}
//	if(pid->inter > his_inter_max) his_inter_max = pid->inter;
//	if(pid->err > his_err_max) his_err_max = pid->err;
//	if(pid->inter>inter_max) pid->inter = inter_max;
//	if(pid->inter<-inter_max) pid->inter = -inter_max;
	pid->pwm = pid->err * pid->kp + pid->inter * pid->ki;
	return pid->pwm;
}




