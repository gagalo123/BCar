#ifndef __MPUI2C__H
#define __MPUI2C__H
#include"run_main.h"
void MPU_IIC_Delay(void);								//IIC延时2ms函数
void MPU_IIC_Init(void);                //初始化IIC的IO口				 
void MPU_IIC_Start(void);								//发送IIC开始信号
void MPU_IIC_Stop(void);	  						//发送IIC停止信号
void MPU_IIC_Send_Byte(u8 txd);					//IIC发送一个字节
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 MPU_IIC_Wait_Ack(void); 							//IIC等待ACK信号
void MPU_IIC_Ack(void);									//IIC发送ACK信号
void MPU_IIC_NAck(void);								//IIC不发送ACK信号
#endif