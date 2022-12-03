#include"motor.h"
#include"stm32f1xx_hal.h"
#include"run_main.h"
#include"angle.h"
#include"speed.h"
#include"tim.h"
#include"gpio.h"
#include"protool.h"
int MotorMode;
int AngleMode;
int SpeedMode;
	const double ag_kp = 560*0.6;
	const double ag_kd = 1.8*0.6;
	
	
  const double sp_kp = 610;
//	const double sp_ki = 1.5;
	

	
	const double turn_kp = 1;
	const double turn_kd = 0.1;
	
	
//	const double turn_kp = 0;
//	const double turn_kd = 0;
//	const double ag_kp = 0;
//	const double ag_kd = 0;
// const double sp_kp = 0;
//	const double sp_ki = 0;
///*
//    速度控制：
//*/


/*
2950
*/
void MotorInit(void) {
	

	HAL_TIM_Base_Start_IT(&htim4);
	AngleControl_Init(ag_kp, ag_kd);
	SpeedControl_Init(&PID_speed, 0,sp_kp, sp_kp/200);
	TurnControl_Init(turn_kp, turn_kd);
	L_OFF;
	R_OFF;
	

	HAL_TIM_Base_Start_IT(&h_encoder_l);
	HAL_TIM_Encoder_Start_IT(&h_encoder_l, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start_IT(&h_pwm_l, TIM_CHANNEL_1);
	
	
  HAL_TIM_Base_Start_IT(&h_encoder_r);
  HAL_TIM_Encoder_Start_IT(&h_encoder_r, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start_IT(&h_pwm_r, TIM_CHANNEL_2);
	
	MotorMode = MOTOR_ON;
	AngleMode = ANGLE_ON;
	SpeedMode = SPEED_ON;
	L_F;
	R_F;
}
double TmpTargetVal;
double PID_TurnValue;
void MotorChangeSpeed(float sp)
{
	TmpTargetVal = sp;
}
void MotorTurnRight(double TurnValue) //右转
{
	
	PID_TurnValue = TurnValue *(TmpTargetVal>=0?-1:1);
	turnning = 1;
}
void MotorTurnLeft(double TurnValue) //左转
{
	PID_TurnValue = -TurnValue *(TmpTargetVal>=0?-1:1);
	turnning = 1;
}
void MotorTurnForward(void)
{
			moving = 1;
			PID_speed.target_val = TmpTargetVal;
	    L_F;
			R_F;
}
void MotorTurnBackward(void)
{
	moving = -1;
	turnning = 0;
			PID_speed.target_val = -TmpTargetVal;
			L_B;
			R_B;
}
void MotorTurnStill(void)
{
	moving = 0;
	turnning = 0;
	PID_speed.target_val = 0;
//			L_F;
//			R_F;
//			TurnValue = 0;
}
void MotorTurnResume(void)
{
			PID_speed.target_val = TmpTargetVal;
//			L_F;
//			R_F;
//			TurnValue = 0;
}
void MotorTurnSLeft(double TurnValue)
{
	MotorTurnLeft(TurnValue);
}
void MotorTurnSRight(double TurnValue)
{
	MotorTurnRight(TurnValue);
}
//int motor_mode;

//enCarState car_state;
//PIDSTRUCT pidl, pidr;
//void PID_Param_Init(PIDSTRUCT *pid, double val){
//    printf("PID_Init...\n");
//    pid->target_val = val;
//    pid->err =
//    pid->err_last =
//    pid->inter =
//    pid->pwm = 0;

//    pid->kp = 300;
//    pid->ki = 300;
//    pid->kd = 100;
//}



//void PID_Init(double val_l, double val_r ) {
//    PID_Param_Init(&pidl, val_l);
//    PID_Param_Init(&pidr, val_r);
//}
//void PID_DeInit(){
//    PID_Param_Init(&pidl, 0);
//    PID_Param_Init(&pidr, 0);
//}

//void set_p_i_d(PIDSTRUCT *pid, PID_V p, PID_V i, PID_V d){
//    pid->kp = p;
//    pid->ki = i;
//    pid->kd = d;
//}
////PID_V PID_readlize(PIDSTRUCT *pid, PID_V val){
////    pid->err = pid->target_val - pid->actural_val;
////    pid->inter += pid->err;
////    pid->actural_val = pid->kp * pid->err
////                    +pid->ki * pid->inter
////                    +pid->kd*(pid->err - pid->err_last);
////    pid->err_last = pid->err;
////    return pid->actural_val;
////}
//int motor_incremental_pi(PIDSTRUCT *pid, double now_v) {
//	int32_t tmp = pid->pwm;
//	pid->err = pid->target_val - now_v;
//	tmp +=pid->kp *(pid->err - pid->err_last)+
//				pid->ki * pid->err;
//	pid->err_last = pid->err;
//	if(tmp > 65535) tmp = 65535;
//	if(tmp < 0) tmp = 0;
//	printf("tar=%f\n", pid->target_val);
//	return pid->pwm = tmp;
//}
//void PID_set_pwm_l(PID_V val) {
//   PWML = val; 
//}







//#define BLA_ON HAL_GPIO_WritePin(BLA_GPIO_Port, BLA_Pin, GPIO_PIN_SET)
//#define BLA_OFF HAL_GPIO_WritePin(BLA_GPIO_Port, BLA_Pin, GPIO_PIN_RESET)
//#define BLB_ON HAL_GPIO_WritePin(BLB_GPIO_Port, BLB_Pin, GPIO_PIN_SET)
//#define BLB_OFF HAL_GPIO_WritePin(BLB_GPIO_Port, BLB_Pin, GPIO_PIN_RESET)
//#define BL_F BLA_ON;BLB_OFF;
//#define BL_B BLA_OFF;BLB_ON;
//#define BL_OFF BLA_OFF;BLB_OFF;

//#define BRA_ON HAL_GPIO_WritePin(BRA_GPIO_Port, BRA_Pin, GPIO_PIN_SET)
//#define BRA_OFF HAL_GPIO_WritePin(BRA_GPIO_Port, BRA_Pin, GPIO_PIN_RESET)
//#define BRB_ON HAL_GPIO_WritePin(BRB_GPIO_Port, BRB_Pin, GPIO_PIN_SET)
//#define BRB_OFF HAL_GPIO_WritePin(BRB_GPIO_Port, BRB_Pin, GPIO_PIN_RESET)
//#define BR_F BRA_ON;BRB_OFF
//#define BR_B BRA_OFF;BRB_ON
//#define BR_OFF BRA_OFF;BRB_OFF
//#define L_OFF BL_OFF
//#define R_OFF BR_OFF
//#define L_F BL_F
//#define R_F BR_F
//#define L_B BL_B
//#define R_B BR_B
//VSTRUCT vl, vr;
//uint16_t v_cnt;
///*
// * 0<= val_l,val_r < 65536
// * 1Mhz
// * 1duty = 10000/1M=0.01
// *
// * */
//void motor_init(double val_l, double val_r){

//	vl.last_h = vr.last_h = vl.last_l = vr.last_l = 0;
//	vl.now_h = vr.now_h = vl.now_l = vr.now_l = 0;
//	L_OFF;
//	R_OFF;
//	motor_mode = MOTOR_OFF;
//	PID_Init(val_l, val_r);

//	HAL_TIM_Base_Start_IT(&htim_encoder_l);
//	HAL_TIM_Encoder_Start_IT(&htim_encoder_l, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Start_IT(&htim_pwm_l, TIM_CHANNEL_1);
//    //PWML = val_l;

//    HAL_TIM_Base_Start_IT(&htim_encoder_r);
//    HAL_TIM_Encoder_Start_IT(&htim_encoder_r, TIM_CHANNEL_ALL);
////    HAL_TIM_Encoder_Start_IT(&htim_encoder_r, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Start_IT(&htim_pwm_r, TIM_CHANNEL_4);
//    //PWMR = val_r;


////    motor_mode = MOTOR_ON;
//}
//void motor_set_speed(double v, int id) {
//	if(id == 0) {
//		pidl.target_val = v;
//	} else if(id == 1) {
//		pidr.target_val = v;
//	}
//}



//void motor_power_off(void){
//	PID_DeInit();
//    HAL_TIM_Encoder_Stop_IT(&htim_encoder_l, TIM_CHANNEL_ALL);
////    HAL_TIM_PWM_Stop_IT(&htim_encoder_l, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Stop_IT(&htim_pwm_l, TIM_CHANNEL_3);


//    HAL_TIM_Encoder_Stop_IT(&htim_encoder_r, TIM_CHANNEL_ALL);
////    HAL_TIM_PWM_Stop_IT(&htim_encoder_r, TIM_CHANNEL_2);
//    HAL_TIM_PWM_Stop_IT(&htim_pwm_r, TIM_CHANNEL_4);

//    motor_mode = MOTOR_OFF;
//}
//void motor_turn_right(void) {
//    L_F;
//    R_OFF;
//	motor_mode = MOTOR_ON;
//    return;
//}
//void motor_turn_left(void){
//    L_OFF;
//    R_F;
//	motor_mode = MOTOR_ON;
//    return;
//}
//void motor_turn_pause(void) {
//	motor_mode = MOTOR_OFF;
//    HAL_TIM_Encoder_Stop_IT(&htim_encoder_l, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Stop_IT(&htim_pwm_l, TIM_CHANNEL_3);

//    HAL_TIM_Encoder_Stop_IT(&htim_encoder_r, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Stop_IT(&htim_pwm_r, TIM_CHANNEL_4);

//    L_OFF;
//    R_OFF;
//    return;
//}
//void motor_turn_resume(void) {
//    HAL_TIM_Encoder_Start_IT(&htim_encoder_l, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Start_IT(&htim_pwm_l, TIM_CHANNEL_3);

//    HAL_TIM_Encoder_Start_IT(&htim_encoder_r, TIM_CHANNEL_ALL);
//    HAL_TIM_PWM_Start_IT(&htim_pwm_r, TIM_CHANNEL_4);
//    L_F;
//    R_F;
//    HAL_Delay(2000); // wait motor to resume
//    motor_mode = MOTOR_ON;
//    return;
//}
//void motor_turn_off(void) {
//   PID_DeInit();
//}
//void motor_turn_forward(void) {
//    L_F;
//    R_F;
//	motor_mode = MOTOR_ON;
//    return;
//}
//void motor_turn_backward(void) {
//    L_B;
//    R_B;
//	motor_mode = MOTOR_ON;
//    return;
//}
//void motor_turn_tleft(void){
//	L_B;
//	R_F;
//	motor_mode = MOTOR_ON;
//}
//void motor_turn_tright(void){
//	L_F;
//	R_B;
//	motor_mode = MOTOR_ON;
//}
