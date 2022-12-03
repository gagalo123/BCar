#ifndef __MOTOR__H__
#define __MOTOR__H__

extern int MotorMode;
extern int AngleMode;
extern int SpeedMode;
extern double PID_TurnValue;
extern const double turn_kp;
extern const double turn_kd;	
extern const double ag_kp;
extern const double ag_kd;
extern const double sp_kp;
extern double TmpTargetVal;
#define MOTOR_ON 1
#define MOTOR_OFF 0
#define ANGLE_ON 1
#define ANGLE_OFF 0
#define SPEED_ON 1
#define SPEED_OFF 0

#define BLA_ON HAL_GPIO_WritePin(BLA_GPIO_Port, BLA_Pin, GPIO_PIN_SET)
#define BLA_OFF HAL_GPIO_WritePin(BLA_GPIO_Port, BLA_Pin, GPIO_PIN_RESET)
#define BLB_ON HAL_GPIO_WritePin(BLB_GPIO_Port, BLB_Pin, GPIO_PIN_SET)
#define BLB_OFF HAL_GPIO_WritePin(BLB_GPIO_Port, BLB_Pin, GPIO_PIN_RESET)
#define BL_F BLA_ON;BLB_OFF;
#define BL_B BLA_OFF;BLB_ON;
#define BL_OFF BLA_OFF;BLB_OFF;

#define BRA_ON HAL_GPIO_WritePin(BRA_GPIO_Port, BRA_Pin, GPIO_PIN_SET)
#define BRA_OFF HAL_GPIO_WritePin(BRA_GPIO_Port, BRA_Pin, GPIO_PIN_RESET)
#define BRB_ON HAL_GPIO_WritePin(BRB_GPIO_Port, BRB_Pin, GPIO_PIN_SET)
#define BRB_OFF HAL_GPIO_WritePin(BRB_GPIO_Port, BRB_Pin, GPIO_PIN_RESET)
#define BR_F BRA_ON;BRB_OFF
#define BR_B BRA_OFF;BRB_ON
#define BR_OFF BRA_OFF;BRB_OFF
#define L_OFF BL_OFF
#define R_OFF BR_OFF

#define L_F BL_F
#define R_F BR_F
#define L_B BL_B
#define R_B BR_B
void MotorInit(void);
void MotorTurnRight(double TurnValue);
void MotorTurnLeft(double TurnValue);
void MotorTurnForward(void);
void MotorTurnBackward(void);
void MotorTurnResume(void);
void MotorTurnSLeft(double TurnValue);
void MotorTurnSRight(double TurnValue);
void MotorChangeSpeed(float sp);
void MotorTurnStill(void);
//typedef int32_t PID_V;

//#define MOTOR_ON 1
//#define MOTOR_OFF 0
//#define MOTOR_FORWARD 2
//#define MOTOR_BACKWARD 3
//#define PWML TIM1->CCR1
//#define PWMR TIM1->CCR4
////extern VSTRUCT vl, vr;
//typedef struct {
//    double target_val;
//    int pwm;
//    double err;
//    double err_last;
//    double kp,ki, kd;
//    double inter;
//}PIDSTRUCT;
////void PID_DeInit();
////void PID_Init(PID_V val_l, PID_V val_r );
////PID_V PID_readlize_l(PID_V val);
////void PID_set_pwm_l(PID_V val);
////PID_V PID_readlize_r(PID_V val);
////void PID_set_pwm_r(PID_V val);
////void PID_set_target_l(PID_V val);
////void PID_set_target_r(PID_V val);

//void motor_init(double val_l, double val_r);
//void motor_power_off(void);

//void motor_turn_right(void);
//void motor_turn_left(void);
//void motor_turn_forward(void);
//void motor_turn_backward(void);
//void motor_turn_pause(void);
//void motor_turn_resume(void);
//void motor_turn_tleft(void);
//void motor_turn_tright(void);

//int motor_incremental_pi(PIDSTRUCT *pid, double now_v);
//void motor_set_speed(double v, int id); //rmp  id=0 L   id=1 R


//typedef enum {

//  enSTOP,
//  enRESUME,
//  enFORWARD,
//  enBACK,
//  enLEFT,
//  enRIGHT,
//  enTLEFT,
//  enTRIGHT

//}enCarState;
//extern enCarState car_state;

#endif
