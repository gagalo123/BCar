#include"mpui2c.h"
#include"run_main.h"

#define SCL_H  GPIOB->BSRR = GPIO_PIN_10
#define SCL_L  GPIOB->BRR = GPIO_PIN_10

#define SDA_H GPIOB->BSRR = GPIO_PIN_11
#define SDA_L GPIOB->BRR = GPIO_PIN_11

#define SDA_R (GPIOB->IDR & GPIO_PIN_11)

#define SDA_OUT() {GPIOB->CRH &= 0xFFFF0FFF;GPIOB->CRH |= 3 << 12;}
#define SDA_IN() {GPIOB->CRH &= 0xFFFF0FFF; GPIOB->CRH |= 8 << 12;}
void MPU_IIC_Delay(void)
{
	Delay_us(2);
}


/**********************************************
�������ƣ�MPU_IIC_Start
�������ܣ�MPU IIC������ʼ�ź�
������������
��������ֵ����
**********************************************/
void MPU_IIC_Start(void)
{
	SDA_OUT();
	SDA_H;
	SCL_H;
	MPU_IIC_Delay();
	SDA_L;
	MPU_IIC_Delay();
	SCL_L;
}

/**********************************************
�������ƣ�MPU_IIC_Stop
�������ܣ�MPU IIC����ֹͣ�ź�
������������
��������ֵ����
**********************************************/
void MPU_IIC_Stop(void)
{
	SDA_OUT();
	SCL_L;
	SDA_L;
	MPU_IIC_Delay();
	SCL_H;
	SDA_H;
	MPU_IIC_Delay();
}

/**********************************************
�������ƣ�MPU_IIC_Wait_Ack
�������ܣ�MPU IIC�ȴ��źŵ���
������������
��������ֵ��1:����Ӧ���źųɹ�  0:����Ӧ���ź�ʧ��
**********************************************/
u8 MPU_IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();
	SDA_H;MPU_IIC_Delay();
	SCL_H;MPU_IIC_Delay();
	while(SDA_R)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MPU_IIC_Stop();
			return 1;
		}
	}
	SCL_L;//ʱ�����0
	return 0;
} 

/**********************************************
�������ƣ�MPU_IIC_Ack
�������ܣ�MPU IIC����Ӧ���ź�
������������
��������ֵ����
**********************************************/
void MPU_IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
	SDA_L;
	MPU_IIC_Delay();
	SCL_H;
	MPU_IIC_Delay();
	SCL_L;
}

/**********************************************
�������ƣ�MPU_IIC_NAck
�������ܣ�MPU IIC������Ӧ���ź�
������������
��������ֵ����
**********************************************/   
void MPU_IIC_NAck(void)
{
	SCL_L;
	SDA_OUT();
	SDA_H;
	MPU_IIC_Delay();
	SCL_H;
	MPU_IIC_Delay();
	SCL_L;
}

/**********************************************
�������ƣ�MPU_IIC_Send_Byte
�������ܣ�MPU IIC����һ���ֽ�
����������txd��Ҫ���͵�����
��������ֵ����
ע�⣺IIC�����ֽ���һ��һ��λ���͵ģ�����һ���ֽ���Ҫ���Ͱ˴�
**********************************************/
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   	    
		SDA_OUT();
    SCL_L;		//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
			if((txd&0x80)>>7) SDA_H;
			else
					SDA_L;
        txd<<=1; 	  
		    SCL_H;
		    MPU_IIC_Delay(); 
		    SCL_L;
		    MPU_IIC_Delay();
    }	 
} 	    

/**********************************************
�������ƣ�MPU_IIC_Read_Byte
�������ܣ�MPU IIC��ȡһ���ֽ�
����������ack: 1,����ACK   0,����NACK 
��������ֵ�����յ�������
ע�⣺IIC��ȡ�ֽ���һ��һ��λ��ȡ�ģ���ȡһ���ֽ���Ҫ��ȡ�˴�
**********************************************/ 
u8 MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++)
	  {
        SCL_L;
        MPU_IIC_Delay();
				SCL_H;
        receive<<=1;
        if(SDA_R)receive++;   //�������������
				MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();   //����nACK
    else
        MPU_IIC_Ack();    //����ACK   
    return receive;
}
