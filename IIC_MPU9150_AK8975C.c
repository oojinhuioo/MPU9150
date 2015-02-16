/*
******************************************************************
* �ļ���IIC_MPU9150_AK8975C.c
* �༭����Ƕ�Ƽ�
* ���ڣ�2013/12/16
* �޸ģ�
* �汾��V1.0
* ������IIC MPU9150����Ŵ���������������ʵ�֡�
******************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "IIC_MPU9150_AK8975C.h"
#include "IIC_MPU6050.h"             //�����������е�IIC��������
#include <math.h>
#include <stdio.h>

float ASAX_factor=0.0,ASAY_factor=0.0,ASAZ_factor=0.0;

/*******************************************************************************
* Function Name  : void AK8975C_SetMode(uint8_t mode)
* Description    : ����ģʽ��
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
* Description    : ��ʼ��AK8975C��ģʽ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AK8975C_Init(void)
{
	uint8_t tmp[3];
	// MPU6050_Init�������Ѿ�ʹ��AUXI2C��pass-throughģʽ��MCU����ֱ�ӿ���AK8975C
	AK8975C_SetMode(_AKM_cMode_FuseROMacc);  //�ֽ�sensitivity adjustment values������
	I2C_BufferRead(MPU9150_AK8975C_ADDR,tmp,MPU9150_AK8975C_ASAX,3);
	ASAX_factor = (tmp[0]-128)*0.5/128+1.0;
	ASAY_factor = (tmp[1]-128)*0.5/128+1.0;
	ASAZ_factor = (tmp[2]-128)*0.5/128+1.0;
	AK8975C_SetMode(_AKM_cMode_SglMeas);
		
}

/*******************************************************************************
* Function Name  : uint8_t AK8975C_SelfTest(void)
* Description    : ��AK8975C�����Բ⡣
* Input          : None
* Output         : None
* Return         : �Բ�״̬
*                  0�� �Բ���ͨ��
*                  x�� ��Ӧ��bit��ʾ����δͨ���Լ�
*******************************************************************************/
uint8_t AK8975C_SelfTest(void)
{
	uint8_t tmp[6];
	uint8_t tries=10,result=0;
	int16_t data;

	AK8975C_SetMode(_AKM_cMode_PwrDown);
	I2C_DEV_ByteWrite(MPU9150_AK8975C_ADDR, _AKM_ST_EN, MPU9150_AK8975C_ASTC);	     //����selftest
	AK8975C_SetMode(_AKM_cMode_SelfTest);										     //����selftest mode
	do{
		if( I2C_DEV_ByteRead(MPU9150_AK8975C_ADDR,MPU9150_AK8975C_ST1)&_AKM_DRDY )	 //�������׼�����
			break;
	}while(tries--);

	I2C_DEV_BufferRead(MPU9150_AK8975C_ADDR,tmp,MPU9150_AK8975C_HXL,6);	                 //��Self Test�����ݶ�����

	//���·�������MPU9150 Product Specification Revision 4.0  5/14/2012	Page 13	���������ݽ����Լ�
	data = (int16_t)(tmp[1]<<8) | tmp[0];
	if((data > 100) || (data < -100))
		result |= 0x01;
	data = (int16_t)(tmp[3]<<8) | tmp[2];
	if((data > 100) || (data < -100))
		result |= 0x02;
	data = (int16_t)(tmp[5]<<8) | tmp[4];
	if((data > -300) || (data < -1000))
		result |= 0x04; 

	I2C_DEV_ByteWrite(MPU9150_AK8975C_ADDR, _AKM_ST_DIS, MPU9150_AK8975C_ASTC);	     //��ֹselftest
	AK8975C_SetMode(_AKM_cMode_PwrDown);
	return result;
}

/*******************************************************************************
* Function Name  : void AK8975C_GetMagnet(float* mx, float* my, float* mz, int16_t* int_mx, int16_t* int_my, int16_t* int_mz)
* Description    : ��ȡ��У�����ݣ���󴫵ݳ�����
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
		I2C_DEV_BufferRead(MPU9150_AK8975C_ADDR, m_buff, MPU9150_AK8975C_HXL, 6); //����ͳ�����һ����
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
