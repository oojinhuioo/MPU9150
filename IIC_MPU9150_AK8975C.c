/*
******************************************************************
* 文件：IIC_MPU9150_AK8975C.c
* 编辑：合嵌科技
* 日期：2013/12/16
* 修改：
* 版本：V1.0
* 描述：IIC MPU9150三轴磁传感器操作函数的实现。
******************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "IIC_MPU9150_AK8975C.h"
#include "IIC_MPU6050.h"             //用于引用其中的IIC操作函数
#include <math.h>
#include <stdio.h>

float ASAX_factor=0.0,ASAY_factor=0.0,ASAZ_factor=0.0;

/*******************************************************************************
* Function Name  : void AK8975C_SetMode(uint8_t mode)
* Description    : 设置模式。
* Input          : mode: _cMode_PwrDown _cMode_SglMeas _cMode_SelfTest _cMode_FuseROMacc
* Output         : None
* Return         : None
*******************************************************************************/
__inline void AK8975C_SetMode(uint8_t mode)
{
	I2C_DEV_ByteWrite(MPU9150_AK8975C_ADDR, mode, MPU9150_AK8975C_CNTL); 	
}
/*******************************************************************************
* Function Name  : void AK8975C_Init(void)
* Description    : 初始化AK8975C的模式。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AK8975C_Init(void)
{
	uint8_t tmp[3];
	// MPU6050_Init函数中已经使能AUXI2C的pass-through模式，MCU可以直接控制AK8975C
	AK8975C_SetMode(_AKM_cMode_FuseROMacc);  //现将sensitivity adjustment values读出来
	I2C_BufferRead(MPU9150_AK8975C_ADDR,tmp,MPU9150_AK8975C_ASAX,3);
	ASAX_factor = (tmp[0]-128)*0.5/128+1.0;
	ASAY_factor = (tmp[1]-128)*0.5/128+1.0;
	ASAZ_factor = (tmp[2]-128)*0.5/128+1.0;
	AK8975C_SetMode(_AKM_cMode_SglMeas);
		
}

/*******************************************************************************
* Function Name  : uint8_t AK8975C_SelfTest(void)
* Description    : 对AK8975C进行自测。
* Input          : None
* Output         : None
* Return         : 自测状态
*                  0： 自测试通过
*                  x： 相应的bit表示哪轴未通过自检
*******************************************************************************/
uint8_t AK8975C_SelfTest(void)
{
	uint8_t tmp[6];
	uint8_t tries=10,result=0;
	int16_t data;

	AK8975C_SetMode(_AKM_cMode_PwrDown);
	I2C_DEV_ByteWrite(MPU9150_AK8975C_ADDR, _AKM_ST_EN, MPU9150_AK8975C_ASTC);	     //允许selftest
	AK8975C_SetMode(_AKM_cMode_SelfTest);										     //进入selftest mode
	do{
		if( I2C_DEV_ByteRead(MPU9150_AK8975C_ADDR,MPU9150_AK8975C_ST1)&_AKM_DRDY )	 //如果数据准备完毕
			break;
	}while(tries--);

	I2C_DEV_BufferRead(MPU9150_AK8975C_ADDR,tmp,MPU9150_AK8975C_HXL,6);	                 //将Self Test的数据读出来

	//以下方法按照MPU9150 Product Specification Revision 4.0  5/14/2012	Page 13	给出的数据进行自检
	data = (int16_t)(tmp[1]<<8) | tmp[0];
	if((data > 100) || (data < -100))
		result |= 0x01;
	data = (int16_t)(tmp[3]<<8) | tmp[2];
	if((data > 100) || (data < -100))
		result |= 0x02;
	data = (int16_t)(tmp[5]<<8) | tmp[4];
	if((data > -300) || (data < -1000))
		result |= 0x04; 

	I2C_DEV_ByteWrite(MPU9150_AK8975C_ADDR, _AKM_ST_DIS, MPU9150_AK8975C_ASTC);	     //禁止selftest
	AK8975C_SetMode(_AKM_cMode_PwrDown);
	return result;
}

/*******************************************************************************
* Function Name  : void AK8975C_GetMagnet(float* mx, float* my, float* mz, int16_t* int_mx, int16_t* int_my, int16_t* int_mz)
* Description    : 读取并校正数据，最后传递出来。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AK8975C_GetMagnet(float* mx, float* my, float* mz, int16_t* i_mx, int16_t* i_my, int16_t* i_mz)
{
	uint8_t m_buff[6],i;
	int16_t tmp_mx, tmp_my, tmp_mz;	
	
	//I2C_DEV_BufferRead(MPU9150_AK8975C_ADDR,&tmp,MPU9150_AK8975C_ST1,1);
	//if(tmp)
	//{
		//AK8975C_SetMode(_AKM_cMode_SglMeas);
		I2C_DEV_BufferRead(MPU9150_AK8975C_ADDR, m_buff, MPU9150_AK8975C_HXL, 6); //问题就出在这一段中
		/*for(i=0;i<6;i++)
		{
			m_buff[i] = I2C_DEV_ByteRead(MPU9150_AK8975C_ADDR,MPU9150_AK8975C_HXL+i);
			delay_msec(1);
		}*/
		*i_mx = tmp_mx = ( ((int16_t)m_buff[1] << 8 ) | ((int16_t)m_buff[0]));
		*i_my = tmp_my = ( ((int16_t)m_buff[3] << 8 ) | ((int16_t)m_buff[2]));
		*i_mz = tmp_mz = ( ((int16_t)m_buff[5] << 8 ) | ((int16_t)m_buff[4]));
	 	//*i_mx = tmp_mx;
		//*i_my = tmp_my;
		//*i_mz = tmp_mZ; 
		*mx =  tmp_mx*ASAX_factor;
		*my =  tmp_my*ASAY_factor;
		*mz =  tmp_mz*ASAZ_factor;
	//}		
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
