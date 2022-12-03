#include"angle.h"
#include"stm32f1xx_hal.h"
#include"run_main.h"
#include"protool.h"
#include<math.h>
#include"motor.h"
/*
    速度控制：
*/
/*
 * 0<= val_l,val_r < 65536
 * 1Mhz
 * 1duty = 10000/1M=0.01
 *
 * */
PID_ANGLE_STRUCT PID_angle;
PID_TRUN_STRUCT PID_turn;


void AngleControl_Init(double kp, double kd) {
	PID_angle.pwm = 0;
	PID_angle.err = 0;
	PID_angle.kp = kp;
	PID_angle.kd = kd;
}


int AngleControl_update(double NowAngle, double NowGyro) {
	PID_angle.err = 0 + (-NowAngle) ;
	PID_angle.pwm = PID_angle.kp * PID_angle.err + PID_angle.kd * -NowGyro;
	return PID_angle.pwm;
}
void TurnControl_Init(double kp, double kd)
{
	PID_turn.kd = kd;
	PID_turn.kp = kp;
	PID_turn.pwm = 0;
	PID_turn.err = 0;
}

int TurnControlUpdate(int l, int r, short gyro)
{
	static int bias;
//	static const int Turn_Amplitude = 3000;
	if(turnning)
	{
			bias = 0;
			return PID_TurnValue;
	} else
	{
		bias += l - r;
//		if(bias > Turn_Amplitude)
//			bias = Turn_Amplitude;
//		if(bias < -Turn_Amplitude)
//			bias = -Turn_Amplitude;
	}
	PID_turn.pwm = bias * PID_turn.kp -  gyro * PID_turn.kd;
	return PID_turn.pwm;
}
