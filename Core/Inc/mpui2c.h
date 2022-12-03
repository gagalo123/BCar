#ifndef __MPUI2C__H
#define __MPUI2C__H
#include"run_main.h"
void MPU_IIC_Delay(void);								//IIC��ʱ2ms����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);								//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  						//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(u8 txd);					//IIC����һ���ֽ�
u8 MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 MPU_IIC_Wait_Ack(void); 							//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);									//IIC����ACK�ź�
void MPU_IIC_NAck(void);								//IIC������ACK�ź�
#endif