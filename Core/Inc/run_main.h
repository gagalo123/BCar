#ifndef __RUN__MAIN__H
#define __RUN__MAIN__H
#include<stdio.h>
#include"usart.h"
#include"i2c.h"
#define GNU_TO_UART


typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;


extern float pitch,roll,yaw;
extern double ObstacleDistance;
extern u8 ObstacleMeasureing;
extern double totDistance;
extern double aveSpeed;
#define hi2c_oled hi2c1
#define MPU_hi2c hi2c2
#define h_encoder_l htim2
#define h_encoder_r htim3
#define h_pwm_l htim1
#define h_pwm_r htim1
#define PWM_MAXRANGE 4999

#define TYRE_R 3.25
#define PWML TIM1->CCR1
#define PWMR TIM1->CCR2
int run_main(void);
void SendUp(void);
void ObstacleMeasure(void);


/*Base part2*/
void Delay_us(u16 tim);
#ifdef GNU_TO_UART
#define IO_UART1 huart1
#define IO_UART2 huart2
int fputc(int ch, FILE *f);
int fgetc(FILE *f);
#endif

#endif