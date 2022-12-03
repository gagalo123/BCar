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
函数名称：MPU_IIC_Start
函数功能：MPU IIC发送起始信号
函数参数：无
函数返回值：无
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
函数名称：MPU_IIC_Stop
函数功能：MPU IIC发送停止信号
函数参数：无
函数返回值：无
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
函数名称：MPU_IIC_Wait_Ack
函数功能：MPU IIC等待信号到来
函数参数：无
函数返回值：1:接收应答信号成功  0:接收应答信号失败
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
	SCL_L;//时钟输出0
	return 0;
} 

/**********************************************
函数名称：MPU_IIC_Ack
函数功能：MPU IIC产生应答信号
函数参数：无
函数返回值：无
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
函数名称：MPU_IIC_NAck
函数功能：MPU IIC不产生应答信号
函数参数：无
函数返回值：无
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
函数名称：MPU_IIC_Send_Byte
函数功能：MPU IIC发送一个字节
函数参数：txd：要发送的数据
函数返回值：无
注意：IIC发送字节是一个一个位发送的，发送一个字节需要发送八次
**********************************************/
void MPU_IIC_Send_Byte(u8 txd)
{                        
    u8 t;   	    
		SDA_OUT();
    SCL_L;		//拉低时钟开始数据传输
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
函数名称：MPU_IIC_Read_Byte
函数功能：MPU IIC读取一个字节
函数参数：ack: 1,发送ACK   0,发送NACK 
函数返回值：接收到的数据
注意：IIC读取字节是一个一个位读取的，读取一个字节需要读取八次
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
        if(SDA_R)receive++;   //如果读到了数据
				MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();   //发送nACK
    else
        MPU_IIC_Ack();    //发送ACK   
    return receive;
}
