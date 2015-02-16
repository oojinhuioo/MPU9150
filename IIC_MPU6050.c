/*
******************************************************************
* 文件：IIC_MPU6050.c
* 编辑：合嵌科技
* 日期：2012/12/2
* 修改：
* 版本：V1.0
* 描述：IIC MPU6050三轴加速度、三轴陀螺仪传感器操作函数的实现。
******************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "IIC_MPU6050.h"
#include <math.h>
#include <stdio.h>
/* Private function prototypes -----------------------------------------------*/
void I2C_DEV_ByteWrite(uint8_t DevAddr, uint8_t pBuffer, uint8_t WriteAddr);
uint8_t I2C_DEV_ByteRead(uint8_t DevAddr, uint8_t ReadAddr);
/*
******************************************************************
*                         ！！特别注意！！  
*           STM32的IIC接口比较奇怪，对于不同的IIC器件，
*           它的burst read要求的构成方式不一样，例如
*           MMA8451的IIC接口的连读就需要I2C_DEV_BufferRead
*           函数完成，在使用MPU6050的时候请使用I2C_BufferRead
******************************************************************
*/
void I2C_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);
void I2C_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void I2C_DEV_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);
void I2C_DEV_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);

void delay_msec(int loop_times);


/*Global Variables -----------------------------------------------------------*/


/*******************************************************************************
* Function Name  : void MPU6050_Init(void)
* Description    : 初始化MPU6050的模式、时钟、量程和辅I2C等。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MPU6050_Init(void)
{
	//uint8_t temp;
	I2C_DEV_ByteWrite(MPU6050_ADDR, _CLOCK_PLL_XGYRO | _PWR_RUN, MPU6050_PWR_MGMT_1);    //设置时钟并处于运行模式
	delay_msec(1000);
	I2C_DEV_ByteWrite(MPU6050_ADDR, 0x00, MPU6050_PWR_MGMT_2);    //设置时钟并处于运行模式

	I2C_DEV_ByteWrite(MPU6050_ADDR, _gCONFIG_FS_250, MPU6050_GYRO_CONFIG);				 //设置陀螺仪的量程
	I2C_DEV_ByteWrite(MPU6050_ADDR, _aCONFIG_FS_8, MPU6050_ACCEL_CONFIG);				 //设置加速度计的量程
	//不让MPU6050控制AUXI2C，让主控制器的I2C与AUXI2C直通
    I2C_DEV_ByteWrite(MPU6050_ADDR, 0x00, MPU6050_USER_CTRL);
	I2C_DEV_ByteWrite(MPU6050_ADDR, 0x32, MPU6050_INT_PIN_CFG);	 //INT_RD_CLEAR

}

/*******************************************************************************
* Function Name  : uint8_t MPU6050_SelfTest(void)
* Description    : 对MPU6050进行自测。
* Input          : None
* Output         : None
* Return         : 自测状态
*                  0： 自测试通过
*                  x： 表示有x个轴不通过 x=1:6
*******************************************************************************/
uint8_t MPU6050_SelfTest(void)
{
	uint16_t STR_ax,STR_ay,STR_az,STR_gx,STR_gy,STR_gz;              //自测响应
	uint16_t XA_STDis,YA_STDis,ZA_STDis,XG_STDis,YG_STDis,ZG_STDis;	 //自测前各轴数据
   	uint16_t XA_STEn,YA_STEn,ZA_STEn,XG_STEn,YG_STEn,ZG_STEn;		 //自测后各轴数据
	uint8_t	 XA_TEST,YA_TEST,ZA_TEST,XG_TEST,YG_TEST,ZG_TEST;        //FT的幂指数参数
	float FT_XA,FT_YA,FT_ZA,FT_XG,FT_YG,FT_ZG,change;                //各轴FT，需要计算
	uint8_t SELF_TEST_REG[4],a_buff[6],g_buff[6];   //X Y Z A
	uint8_t err=0;
	//保证测试使用的量程
	I2C_DEV_ByteWrite(MPU6050_ADDR, _gCONFIG_FS_250, MPU6050_GYRO_CONFIG);				 //设置陀螺仪的量程
	I2C_DEV_ByteWrite(MPU6050_ADDR, _aCONFIG_FS_8, MPU6050_ACCEL_CONFIG);				 //设置加速度计的量程

	I2C_BufferRead(MPU6050_ADDR, SELF_TEST_REG, MPU6050_SELF_TEST_X, 4); //将自测寄存器里面的数据读出来
	//提取FT的幂指数参数
	XG_TEST	= SELF_TEST_REG[0]&0x1F;
	YG_TEST	= SELF_TEST_REG[1]&0x1F;
	ZG_TEST	= SELF_TEST_REG[2]&0x1F;
	XA_TEST	= ( (SELF_TEST_REG[0]>>3)&0x1C ) | ( (SELF_TEST_REG[3]>>4)&0x03);
	YA_TEST	= ( (SELF_TEST_REG[1]>>3)&0x1C ) | ( (SELF_TEST_REG[3]>>2)&0x03);
	ZA_TEST	= ( (SELF_TEST_REG[2]>>3)&0x1C ) | ( SELF_TEST_REG[3]&0x03);
	//计算FT
	if(XG_TEST == 0) //XG
	{
		err++;	
	}else
	{
		FT_XG = 3275*pow(1.046, XG_TEST - 1);	
	}
	if(YG_TEST == 0) //YG
	{
		err++;	
	}else
	{
		FT_YG = -3275*pow(1.046, YG_TEST - 1);	
	}
	if(ZG_TEST == 0) //ZG
	{
		err++;	
	}else
	{
		FT_ZG = 3275*pow(1.046, ZG_TEST - 1);	
	}
	if(XA_TEST == 0) //XA
	{
		err++;	
	}else
	{
		FT_XA = 1392.64*pow(0.92/0.34, (XA_TEST - 1)/30);	
	}
	if(YA_TEST == 0) //YA
	{
		err++;	
	}else
	{
		FT_YA = 1392.64*pow(0.92/0.34, (YA_TEST - 1)/30);	
	}
	if(ZA_TEST == 0) //ZA
	{
		err++;	
		return err;
	}else
	{
		FT_ZA = 1392.64*pow(0.92/0.34, (ZA_TEST - 1)/30);	
	}

	//读取数据
	I2C_BufferRead(MPU6050_ADDR, a_buff, MPU6050_ACCEL_XOUT_H, 6);     //读取未使能自测前加速度数据
	I2C_BufferRead(MPU6050_ADDR, g_buff, MPU6050_GYRO_XOUT_H, 6); 	   //读取未使能自测前陀螺仪数据	
	//重组数据
	XA_STDis = ( ((int16_t)a_buff[0] << 8 ) | ((int16_t)a_buff[1]));
	YA_STDis = ( ((int16_t)a_buff[2] << 8 ) | ((int16_t)a_buff[3]));
	ZA_STDis = ( ((int16_t)a_buff[4] << 8 ) | ((int16_t)a_buff[5]));
	XG_STDis = ( ((int16_t)g_buff[0] << 8 ) | ((int16_t)g_buff[1]));
	YG_STDis = ( ((int16_t)g_buff[2] << 8 ) | ((int16_t)g_buff[3]));
	ZG_STDis = ( ((int16_t)g_buff[4] << 8 ) | ((int16_t)g_buff[5]));
	//使能自测
	I2C_DEV_ByteWrite(MPU6050_ADDR,  _gCONFIG_FS_250 | _gCONFIG_XYZG_ST, MPU6050_GYRO_CONFIG);	 //使能陀螺仪三轴自测
	I2C_DEV_ByteWrite(MPU6050_ADDR,  _aCONFIG_FS_8 | _aCONFIG_XYZA_ST, MPU6050_ACCEL_CONFIG);	 //使能加速度计三轴自测

	//读取数据
	I2C_BufferRead(MPU6050_ADDR, a_buff, MPU6050_ACCEL_XOUT_H, 6);     //读取使能自测后加速度数据
	I2C_BufferRead(MPU6050_ADDR, g_buff, MPU6050_GYRO_XOUT_H, 6); 	   //读取使能自测后陀螺仪数据
	//重组数据
	XA_STEn = ( ((int16_t)a_buff[0] << 8 ) | ((int16_t)a_buff[1]));
	YA_STEn = ( ((int16_t)a_buff[2] << 8 ) | ((int16_t)a_buff[3]));
	ZA_STEn = ( ((int16_t)a_buff[4] << 8 ) | ((int16_t)a_buff[5]));
	XG_STEn = ( ((int16_t)g_buff[0] << 8 ) | ((int16_t)g_buff[1]));
	YG_STEn = ( ((int16_t)g_buff[2] << 8 ) | ((int16_t)g_buff[3]));
	ZG_STEn = ( ((int16_t)g_buff[4] << 8 ) | ((int16_t)g_buff[5]));
	//关闭自测
	I2C_DEV_ByteWrite(MPU6050_ADDR, _gCONFIG_FS_250, MPU6050_GYRO_CONFIG);				 //设置陀螺仪的量程
	I2C_DEV_ByteWrite(MPU6050_ADDR, _aCONFIG_FS_8, MPU6050_ACCEL_CONFIG);				 //设置加速度计的量程
	//计算STR
	STR_ax = XA_STEn - XA_STDis; 
	STR_ay = YA_STEn - YA_STDis;
	STR_az = ZA_STEn - ZA_STDis;
	STR_gx = XG_STEn - XG_STDis;
	STR_gy = YG_STEn - YG_STDis;
	STR_gz = ZG_STEn - ZG_STDis;
	//计算并判断
	change = ((STR_ax - FT_XA)/FT_XA)*100;
	if((change>-14) | (change<14))
	{
		;
	}else
	{
		err++;
	}
	change = ((STR_ay - FT_YA)/FT_YA)*100;
	if((change>-14) | (change<14))
	{
		;
	}else
	{
		err++;
	}
	change = ((STR_az - FT_XA)/FT_ZA)*100;
	if((change>-14) | (change<14))
	{
		;
	}else
	{
		err++;
	}
	change = ((STR_gx - FT_XG)/FT_XG)*100;
	if((change>-14) | (change<14))
	{
		;
	}else
	{
		err++;
	}
	change = ((STR_gy - FT_YG)/FT_YG)*100;
	if((change>-14) | (change<14))
	{
		;
	}else
	{
		err++;
	}
	change = ((STR_gz - FT_XG)/FT_ZG)*100;
	if((change>-14) | (change<14))
	{
		;
	}else
	{
		err++;
	}
	return err;
}

/*******************************************************************************
* Function Name  : void MPU6050_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
* Description    : 获取六轴传感器数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MPU6050_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{	  
	uint8_t a_buff[6],g_buff[6];  

	//读取数据
	I2C_BufferRead(MPU6050_ADDR, a_buff, MPU6050_ACCEL_XOUT_H, 6);    
	I2C_BufferRead(MPU6050_ADDR, g_buff, MPU6050_GYRO_XOUT_H, 6); 	  

	*ax = ( ((int16_t)a_buff[0] << 8 ) | ((int16_t)a_buff[1]));
	*ay = ( ((int16_t)a_buff[2] << 8 ) | ((int16_t)a_buff[3]));
	*az = ( ((int16_t)a_buff[4] << 8 ) | ((int16_t)a_buff[5]));
	*gx = ( ((int16_t)g_buff[0] << 8 ) | ((int16_t)g_buff[1]));
	*gy = ( ((int16_t)g_buff[2] << 8 ) | ((int16_t)g_buff[3]));
	*gz = ( ((int16_t)g_buff[4] << 8 ) | ((int16_t)g_buff[5]));
}

/*******************************************************************************
* Function Name  : int16_t MPU6050_GetTemperature(void)
* Description    : 获取温度值。
* Input          : None
* Output         : None
* Return         : 温度值
*******************************************************************************/
float MPU6050_GetTemperature(void)
{
	uint8_t buff[2];
	int16_t	temp;
	float retvalue;
	I2C_BufferRead(MPU6050_ADDR, buff, MPU6050_TEMP_OUT_H, 2);
	
	temp = 	( (int16_t)buff[0] << 8  | (int16_t)buff[1] );
	retvalue =  temp/340 + 36.53;
	return retvalue;
}

/*******************************************************************************
* Function Name  : void delay_msec(int loop_times)
* Description    : 软件延时，延时1ms.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void delay_msec(int loop_times)
{
    vu32 loop_i,loop_j;
    for(loop_i=0; loop_i<loop_times; loop_i++)
    {
        for(loop_j=0; loop_j<5500; loop_j++)
        {
          ;
        }
    }
}


//底层函数	这个后面的函数可以适应所有的IIC器件
/*******************************************************************************
* Function Name  : void I2C_DEV_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
* Description    : 向IIC器件写入数个字节。
* Input          : - DevAddr  :	要写入的IIC器件的地址。
*                  - pBuffer : 指向要写入IIC器件的数据的指针。
*                  - WriteAddr : 要写入的IIC器件的内部地址。
*                  - NumByteToWrite: 写入的字节的个数。
* Output         : None
* Return         : None 
*******************************************************************************/
void I2C_DEV_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
 	uint8_t ptr1;
	uint8_t* ptr2;
	ptr1 = WriteAddr;
	ptr2 = pBuffer;

  	while(NumByteToWrite)  
  	{
    	I2C_DEV_ByteWrite(DevAddr, *ptr2, ptr1);
		ptr1++;
		ptr2++;
		NumByteToWrite--;	
  	}
}

/*******************************************************************************
* Function Name  : void I2C_DEV_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
* Description    : 从IIC器件中读出几个数据。
* Input          : - DevAddr  :	要写入的IIC器件的地址。
*                  - pBuffer : 指向存放从IIC器件接收数据缓冲的指针。
*                  - ReadAddr : 从IIC器件读数据的内部地址。
*                  - NumByteToRead : 要读出的数据个数。
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DEV_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  	uint8_t ptr1;
	uint8_t* ptr2;
	ptr1 = ReadAddr;
	ptr2 = pBuffer;

  	while(NumByteToRead)  
  	{	
    	(*ptr2) = I2C_DEV_ByteRead(DevAddr, ptr1);	
		ptr1++;
		ptr2++;
		NumByteToRead--;		
  	}
}

/*******************************************************************************
* Function Name  : void I2C_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
* Description    : 从IIC器件中读出几个数据。
* Input          : - DevAddr  :	要写入的IIC器件的地址。
*                  - pBuffer : 指向存放从IIC器件接收数据缓冲的指针。
*                  - ReadAddr : 从IIC器件读数据的内部地址。
*                  - NumByteToRead : 要读出的数据个数。
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	I2C_GenerateSTART(I2C1, ENABLE);										   //发送起始位
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
  	//如果只读一个数据，在读之前禁止掉应答 
  	if(NumByteToRead==1) 
  	{
    	I2C_AcknowledgeConfig(I2C1, DISABLE);
  	}
  
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);	          //发送IIC器件地址（写）
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  	I2C_SendData(I2C1, ReadAddr);  											  //发送IIC内部地址（写）
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  	 
  	I2C_GenerateSTART(I2C1, ENABLE);										  //再次发送起始条件 
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  	
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);		          //发送IIC器件地址（读）
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  	while(NumByteToRead)  
  	{												   	
    	if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    	{ 
      		if(NumByteToRead == 2)												  //在倒数第二个字节时禁止应答
      		{       	
        		I2C_AcknowledgeConfig(I2C1, DISABLE);
      		}
      		if(NumByteToRead == 1)
      		{      	
        		I2C_GenerateSTOP(I2C1, ENABLE);									  //在最后字节时发送停止条件
      		}
      
      		*pBuffer = I2C_ReceiveData(I2C1);
      		pBuffer++; 
      		NumByteToRead--;    
    	}   
  	}
  	
  	I2C_AcknowledgeConfig(I2C1, ENABLE);										  //再次使能应答，准备下一次接收	
}

/*******************************************************************************
* Function Name  : void I2C_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
* Description    : 向IIC器件写入数个字节。
* Input          : - DevAddr  :	要写入的IIC器件的地址。
*                  - pBuffer : 指向要写入IIC器件的数据的指针。
*                  - WriteAddr : 要写入的IIC器件的内部地址。
*                  - NumByteToWrite: 写入的字节的个数。
* Output         : None
* Return         : None 
*******************************************************************************/
void I2C_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
	I2C_GenerateSTART(I2C1, ENABLE);										 //发送起始位
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
  
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);	         //发送IIC器件地址（写）
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  
 
  	I2C_SendData(I2C1, WriteAddr);  										 //发送IIC内部地址（写）
  	while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  	while(NumByteToWrite--)  
  	{  	
    	I2C_SendData(I2C1, *pBuffer); 									      //发送当前字节
    	pBuffer++; 
    	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  	}

  	I2C_GenerateSTOP(I2C1, ENABLE); 										  //发送停止位 	
}
/*******************************************************************************
* Function Name  : void I2C_DEV_ByteWrite(uint8_t DevAddr, uint8_t pBuffer, uint8_t WriteAddr)
* Description    : 写一个字节到IIC器件的内部寄存器。
* Input          : - DevAddr  :	要写入的IIC器件的地址。
*				   - pBuffer  : 要写入的数据。
*				   - WriteAddr: 要写入的地址。
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DEV_ByteWrite(uint8_t DevAddr, uint8_t pBuffer, uint8_t WriteAddr)
{
	//等到I2C总线不忙
  	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
    
	//产生起始位
  	I2C_GenerateSTART(I2C1, ENABLE);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
  
   	//发送IIC器件的器件地址（写）
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
   
    //发送IIC器件的内部地址（写）
  	I2C_SendData(I2C1, WriteAddr);  
  	while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
   
	//发送数据
  	I2C_SendData(I2C1, pBuffer);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
   
   	//产生停止位
  	I2C_GenerateSTOP(I2C1, ENABLE);
}
/*******************************************************************************
* Function Name  : uint8_t I2C_DEV_ByteRead(uint8_t DevAddr, uint8_t ReadAddr)
* Description    : 从IIC器件寄存器里面读取一个数据。
* Input          : - DevAddr  :	要写入的IIC器件的地址。
*				   - ReadAddr : IIC器件内部寄存器的地址。
* Output         : None
* Return         : 读出的寄存器的数据
*******************************************************************************/
uint8_t I2C_DEV_ByteRead(uint8_t DevAddr, uint8_t ReadAddr)
{
	uint8_t RxData;
	//等到I2C总线不忙
  	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	//产生起始位
  	I2C_GenerateSTART(I2C1, ENABLE);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 

  	//发送IIC器件的器件地址（写）
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	//发送IIC器件的内部地址（写）
  	I2C_SendData(I2C1, ReadAddr);  
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
	//再次产生起始位 
  	I2C_GenerateSTART(I2C1, ENABLE);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
	//发送IIC器件的内部地址（读）
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); 

   	//等到数据接收到 
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	//读出数据
	RxData = I2C_ReceiveData(I2C1);

	//禁止应答
    I2C_AcknowledgeConfig(I2C1, DISABLE);

	//产生停止位
    I2C_GenerateSTOP(I2C1, ENABLE);
	return RxData;
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
