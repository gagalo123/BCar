#include"run_main.h"
#include"mpu6050.h"
#include"inv_mpu.h"
#include"inv_mpu_dmp_motion_driver.h"
#include"oled.h"
#include"motor.h"
#include"speed.h"
#include"dma_scanf.h"
#include "dma_printf.h"
#include"tim.h"
#include"mpui2c.h"
#include"protool.h"
#include"gpio.h"
//MPU6050_t MPU6050;
 float pitch,roll,yaw; 	
double totDistance;
double ObstacleDistance = 99;
double aveSpeed = 0;
void SendUp(void)
{
	char s[123];
	printf("s0,%.2f,0,0,0,%.2f,%u\n", aveSpeed, totDistance, HAL_GetTick());
}
u8 fac_us;
//int Detection_Obstacle(void) {
//    double  tr;
//    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
//    HAL_Delay(1);// 
//    HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
//		HAL_Delay(2);// 
//    if(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)return -1;
//		uint32_t ticks;
//    uint32_t told,tnow=0,tcnt=0;
//    uint32_t reload=SysTick->LOAD;
////    ticks=tim*fac_us; 
//    told=SysTick->VAL; 
//    while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET) {
//			  tnow=SysTick->VAL;
//        if(tnow!=told)
//        {
//            if(tnow<told)tcnt+=told-tnow;
//            else tcnt+=reload-tnow+told;
//            told=tnow;
//        }
//				tr = tcnt / (double)fac_us;
//				if(tr > 500) break;
////				if( HAL_GetTick() - tl > 1)
////				{
////					return 0;
////				}
//    }
//					  tnow=SysTick->VAL;
//        if(tnow!=told)
//        {
//            if(tnow<told)tcnt+=told-tnow;
//            else tcnt+=reload-tnow+told;
//            told=tnow;
//        }
//		tr = tcnt / (double)fac_us;
//		ObstacleDistance = tr * 0.034f;
//		if(tr > 500) 
//			return 0;
//    return 1;
//}
u8 ObstacleMeasureing;
void ObstacleMeasure(void)
{
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	Delay_us(60);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
}

int run_main(void) {
	u8 c;
	int obs_flag;
	fac_us = SystemCoreClock / 1000000;
	int tmp, i;
	dma_printf_init(&huart1);
	dma_scanf_init(&huart1);
	OLED_Init();
	OLED_ColorTurn(0);         
  OLED_DisplayTurn(0);      
	
//	while(1)
//	{
//		MPU_IIC_Start();
//		Delay_us(500);
//		
//	}
//	GPIOB->BRR = GPIO_PIN_15;
//	while(1) {
//		Delay_us(2);
//		GPIOB->BSRR = GPIO_PIN_14;
//		Delay_us(2);
//		GPIOB->BRR = GPIO_PIN_14;
//	}
//	OLED_ShowString(15,0,"Waiting",24,1);
//	OLED_ShowString(0,28,"Initialise",24,1);
//	OLED_Refresh();
	MPU_Init();					       //???MPU6050
	

	
	while(mpu_dmp_init())
 	{
	}
//	OLED_Clear();
//	OLED_Refresh();
	OLED_ShowString(0,0,"PITCH:",12,1);
	OLED_ShowString(0,12,"V:",12,1);
	OLED_ShowString(0,24,"OBS:",12,1);
	OLED_ShowString(0,36,"D:",12,1);
	OLED_ShowString(0,48,"T:",12,1);
	OLED_Refresh();
	//while(1)mpu_dmp_get_data(&pitch,&roll,&yaw);
	MotorInit();
  	while(1)
		{
			if(dma_ring_available(&dsi.rx_ring))
			{
				int begin = 0;
				int end = 0;
				protool_length = 0;
				while(dma_ring_available(&dsi.rx_ring))
				{
					dma_ring_getc(&dsi.rx_ring, &c);
					if(c == '\n') break;
					
					if(begin)
					{
							protool[protool_length++] = c;
					} else begin = 0;
					if(c == '$')
					{
							begin = 1;
					}

					if(c == '#')
					{
							end = 1;
							protool[protool_length-1] = '\0';
					}
				}
				if(end == 1)
				{
						ProcessProtool();
				}
			}
				if((HAL_GetTick()/100) % 2 == 0)ObstacleMeasure();
	OLED_ShowFloat(50, 0,pitch,1,12,1);
//	OLED_ShowFloat(50, 0,yaw,1,12,1);
	OLED_ShowFloat(50, 12,aveSpeed,5,12,1);
	OLED_ShowFloat(50, 24,ObstacleDistance,5,12,1);
//			OLED_ShowString(50, 24, obs_flag == 1 ? "YES" : "NO ", 12, 1);
	OLED_ShowFloat(50, 36,totDistance,5,12,1);
	OLED_ShowNum(50, 48, HAL_GetTick()/ 1000, 5, 12, 1);
	OLED_Refresh();
			if(target.start)
			{
				
				for(i = 0; i < 2; ++i)
				{
					MotorTurnStill();
					double SYaw = yaw;
					double TYaw;
					//anti-clockwise =>+yaw
					if(target.x[i] < 0)
					{
						// anti-clockwise  90
						if(SYaw + 90 > 180)
						{
								MotorTurnLeft(1000);
								while(yaw > 0);
								MotorTurnStill();
								TYaw = -180 + (90 - (180 - SYaw));
						}
						else
						{
							TYaw = SYaw + 90;
						}
						MotorTurnLeft(1000);
						while(yaw < TYaw);
						MotorTurnStill();
										
						
						target.cx = totDistance;
						MotorTurnForward();
						while(totDistance < target.cx - target.x[i]);
						MotorTurnStill();
						
					} 
					else
					{
						if(SYaw - 90 < -180)
						{
							MotorTurnRight(1000);
							while(yaw < 0);
							MotorTurnStill();
							TYaw = 180 - (90 - (SYaw + 180));
						}
						else
						{
							TYaw = SYaw - 90;
						}
						MotorTurnRight(1000);
						while(yaw > TYaw);
						MotorTurnStill();
						
						target.cx = totDistance;
						MotorTurnForward();
						while(totDistance < target.cx + target.x[i]);
						MotorTurnStill();
						
					}
				}
			
				
				
		
				target.start = 0;
			}
				if(pitch > 40 || pitch < -40)
			{
					MotorMode = MOTOR_OFF;
					AngleMode = ANGLE_OFF;
					SpeedMode = SPEED_OFF;
					PWML = PWMR = 0;
			}
		}

			
//		//printf("sizeof %d %d %d %d\n", sizeof (float), sizeof (int), sizeof(double), sizeof(long));
//		//HAL_Delay(10);

//		
////			if((res = mpu_dmp_get_data(&pitch,&roll,&yaw))==0)
////			{
////				
////				
////				OLED_ShowFloat(50, 0,pitch,1,16,1);
////				
////				
////				
////				OLED_Refresh();
////			}else printf("MPU_dmp_get_data error %u\n", res);
//	
//		} 	
//}
		
		}


void Delay_us(u16 tim)
{
uint32_t ticks;
    uint32_t told,tnow,tcnt=0;
    uint32_t reload=SysTick->LOAD;
    ticks=tim*fac_us; 
    told=SysTick->VAL; 
    while(1)
    {
        tnow=SysTick->VAL;
        if(tnow!=told)
        {
            if(tnow<told)tcnt+=told-tnow;
            else tcnt+=reload-tnow+told;
            told=tnow;
            if(tcnt>=ticks)break; 
        }
    };
//	u16 differ = 0xffff - tim - 5;
//	__HAL_TIM_SetCounter(&htim4, differ);
//	//HAL_TIM_Base_Start(&htim4);
//	while(differ < 0xffff - 5)
//	{
//		differ = __HAL_TIM_GET_COUNTER(&htim4);
//	}
	//HAL_TIM_Base_Stop(&htim4);
	
//	u32 Delay = (u32) tim * 72 / 4;
//	do
//	{
//		__NOP();
//	}while(Delay--);
	//__HAL_TIM_SET_COUNTER(&htim4, tim);
	//while(__HAL_TIM_GET_COUNTER(&htim4) < tim);
	return;
}

int fputc(int ch, FILE *f)
{
	//HAL_UART_Transmit(&IO_UART1, (uint8_t *)&ch, 1, 1000);	
	
//	return (ch);
	  dma_printf_putc(ch&0xFF);
		return ch;
}
