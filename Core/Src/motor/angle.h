#ifndef __ANGLE__H__
#define __ANGLE__H__
#include<stdint.h>
 typedef struct {
    int pwm;
    double err;
    double kp,kd;
		double target;
}PID_ANGLE_STRUCT;
  typedef struct {
    int pwm;
    double err;
    double kp,kd;
}PID_TRUN_STRUCT;
extern int TurnValue;
extern PID_ANGLE_STRUCT PID_angle;
extern PID_TRUN_STRUCT PID_turn;
void AngleControl_Init(double kp, double kd);
int AngleControl_update(double NowAngle, double NowGyro);
int TurnControlUpdate(int l, int r, short groy);
void TurnControl_Init(double kp, double kd);
#endif
