#ifndef __SPEED__H__
#define __SPEED__H__


typedef struct {
    int now_l;
}SPEED_MEASURE_STRUCT;
extern SPEED_MEASURE_STRUCT vl, vr;


typedef struct {
	double l;
	double r;
}SPEED_STRUCT;

extern SPEED_STRUCT sp;


 typedef struct {
    double target_val;
    int pwm;
    double kp,ki;
    double inter, err, err_last;
}PID_SPEED_STRUCT;
 

extern PID_SPEED_STRUCT PID_speed;
int SpeedControl_update(PID_SPEED_STRUCT *pid, double NowSpeed);
void SpeedControl_Init(PID_SPEED_STRUCT *pid, float target, double kp, double ki);
#endif
