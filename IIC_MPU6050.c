/*
******************************************************************
* �ļ���IIC_MPU6050.c
* �༭����Ƕ�Ƽ�
* ���ڣ�2012/12/2
* �޸ģ�
* �汾��V1.0
* ������IIC MPU6050������ٶȡ����������Ǵ���������������ʵ�֡�
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
*                         �����ر�ע�⣡��  
*           STM32��IIC�ӿڱȽ���֣����ڲ�ͬ��IIC������
*           ����burst readҪ��Ĺ��ɷ�ʽ��һ��������
*           MMA8451��IIC�ӿڵ���������ҪI2C_DEV_BufferRead
*           ������ɣ���ʹ��MPU6050��ʱ����ʹ��I2C_BufferRead
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
* Description    : ��ʼ��MPU6050��ģʽ��ʱ�ӡ����̺͸�I2C�ȡ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MPU6050_Init(void)
{
	//uint8_t temp;
	I2C_DEV_ByteWrite(MPU6050_ADDR, _CLOCK_PLL_XGYRO | _PWR_RUN, MPU6050_PWR_MGMT_1);    //����ʱ�Ӳ���������ģʽ
	delay_msec(1000);
	I2C_DEV_ByteWrite(MPU6050_ADDR, 0x00, MPU6050_PWR_MGMT_2);    //����ʱ�Ӳ���������ģʽ

	I2C_DEV_ByteWrite(MPU6050_ADDR, _gCONFIG_FS_250, MPU6050_GYRO_CONFIG);				 //���������ǵ�����
	I2C_DEV_ByteWrite(MPU6050_ADDR, _aCONFIG_FS_8, MPU6050_ACCEL_CONFIG);				 //���ü��ٶȼƵ�����
	//����MPU6050����AUXI2C��������������I2C��AUXI2Cֱͨ
    I2C_DEV_ByteWrite(MPU6050_ADDR, 0x00, MPU6050_USER_CTRL);
	I2C_DEV_ByteWrite(MPU6050_ADDR, 0x32, MPU6050_INT_PIN_CFG);	 //INT_RD_CLEAR

}

/*******************************************************************************
* Function Name  : uint8_t MPU6050_SelfTest(void)
* Description    : ��MPU6050�����Բ⡣
* Input          : None
* Output         : None
* Return         : �Բ�״̬
*                  0�� �Բ���ͨ��
*                  x�� ��ʾ��x���᲻ͨ�� x=1:6
*******************************************************************************/
uint8_t MPU6050_SelfTest(void)
{
	uint16_t STR_ax,STR_ay,STR_az,STR_gx,STR_gy,STR_gz;              //�Բ���Ӧ
	uint16_t XA_STDis,YA_STDis,ZA_STDis,XG_STDis,YG_STDis,ZG_STDis;	 //�Բ�ǰ��������
   	uint16_t XA_STEn,YA_STEn,ZA_STEn,XG_STEn,YG_STEn,ZG_STEn;		 //�Բ���������
	uint8_t	 XA_TEST,YA_TEST,ZA_TEST,XG_TEST,YG_TEST,ZG_TEST;        //FT����ָ������
	float FT_XA,FT_YA,FT_ZA,FT_XG,FT_YG,FT_ZG,change;                //����FT����Ҫ����
	uint8_t SELF_TEST_REG[4],a_buff[6],g_buff[6];   //X Y Z A
	uint8_t err=0;
	//��֤����ʹ�õ�����
	I2C_DEV_ByteWrite(MPU6050_ADDR, _gCONFIG_FS_250, MPU6050_GYRO_CONFIG);				 //���������ǵ�����
	I2C_DEV_ByteWrite(MPU6050_ADDR, _aCONFIG_FS_8, MPU6050_ACCEL_CONFIG);				 //���ü��ٶȼƵ�����

	I2C_BufferRead(MPU6050_ADDR, SELF_TEST_REG, MPU6050_SELF_TEST_X, 4); //���Բ�Ĵ�����������ݶ�����
	//��ȡFT����ָ������
	XG_TEST	= SELF_TEST_REG[0]&0x1F;
	YG_TEST	= SELF_TEST_REG[1]&0x1F;
	ZG_TEST	= SELF_TEST_REG[2]&0x1F;
	XA_TEST	= ( (SELF_TEST_REG[0]>>3)&0x1C ) | ( (SELF_TEST_REG[3]>>4)&0x03);
	YA_TEST	= ( (SELF_TEST_REG[1]>>3)&0x1C ) | ( (SELF_TEST_REG[3]>>2)&0x03);
	ZA_TEST	= ( (SELF_TEST_REG[2]>>3)&0x1C ) | ( SELF_TEST_REG[3]&0x03);
	//����FT
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

	//��ȡ����
	I2C_BufferRead(MPU6050_ADDR, a_buff, MPU6050_ACCEL_XOUT_H, 6);     //��ȡδʹ���Բ�ǰ���ٶ�����
	I2C_BufferRead(MPU6050_ADDR, g_buff, MPU6050_GYRO_XOUT_H, 6); 	   //��ȡδʹ���Բ�ǰ����������	
	//��������
	XA_STDis = ( ((int16_t)a_buff[0] << 8 ) | ((int16_t)a_buff[1]));
	YA_STDis = ( ((int16_t)a_buff[2] << 8 ) | ((int16_t)a_buff[3]));
	ZA_STDis = ( ((int16_t)a_buff[4] << 8 ) | ((int16_t)a_buff[5]));
	XG_STDis = ( ((int16_t)g_buff[0] << 8 ) | ((int16_t)g_buff[1]));
	YG_STDis = ( ((int16_t)g_buff[2] << 8 ) | ((int16_t)g_buff[3]));
	ZG_STDis = ( ((int16_t)g_buff[4] << 8 ) | ((int16_t)g_buff[5]));
	//ʹ���Բ�
	I2C_DEV_ByteWrite(MPU6050_ADDR,  _gCONFIG_FS_250 | _gCONFIG_XYZG_ST, MPU6050_GYRO_CONFIG);	 //ʹ�������������Բ�
	I2C_DEV_ByteWrite(MPU6050_ADDR,  _aCONFIG_FS_8 | _aCONFIG_XYZA_ST, MPU6050_ACCEL_CONFIG);	 //ʹ�ܼ��ٶȼ������Բ�

	//��ȡ����
	I2C_BufferRead(MPU6050_ADDR, a_buff, MPU6050_ACCEL_XOUT_H, 6);     //��ȡʹ���Բ����ٶ�����
	I2C_BufferRead(MPU6050_ADDR, g_buff, MPU6050_GYRO_XOUT_H, 6); 	   //��ȡʹ���Բ������������
	//��������
	XA_STEn = ( ((int16_t)a_buff[0] << 8 ) | ((int16_t)a_buff[1]));
	YA_STEn = ( ((int16_t)a_buff[2] << 8 ) | ((int16_t)a_buff[3]));
	ZA_STEn = ( ((int16_t)a_buff[4] << 8 ) | ((int16_t)a_buff[5]));
	XG_STEn = ( ((int16_t)g_buff[0] << 8 ) | ((int16_t)g_buff[1]));
	YG_STEn = ( ((int16_t)g_buff[2] << 8 ) | ((int16_t)g_buff[3]));
	ZG_STEn = ( ((int16_t)g_buff[4] << 8 ) | ((int16_t)g_buff[5]));
	//�ر��Բ�
	I2C_DEV_ByteWrite(MPU6050_ADDR, _gCONFIG_FS_250, MPU6050_GYRO_CONFIG);				 //���������ǵ�����
	I2C_DEV_ByteWrite(MPU6050_ADDR, _aCONFIG_FS_8, MPU6050_ACCEL_CONFIG);				 //���ü��ٶȼƵ�����
	//����STR
	STR_ax = XA_STEn - XA_STDis; 
	STR_ay = YA_STEn - YA_STDis;
	STR_az = ZA_STEn - ZA_STDis;
	STR_gx = XG_STEn - XG_STDis;
	STR_gy = YG_STEn - YG_STDis;
	STR_gz = ZG_STEn - ZG_STDis;
	//���㲢�ж�
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
* Description    : ��ȡ���ᴫ��������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MPU6050_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{	  
	uint8_t a_buff[6],g_buff[6];  

	//��ȡ����
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
* Description    : ��ȡ�¶�ֵ��
* Input          : None
* Output         : None
* Return         : �¶�ֵ
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
* Description    : �����ʱ����ʱ1ms.
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


//�ײ㺯��	�������ĺ���������Ӧ���е�IIC����
/*******************************************************************************
* Function Name  : void I2C_DEV_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
* Description    : ��IIC����д�������ֽڡ�
* Input          : - DevAddr  :	Ҫд���IIC�����ĵ�ַ��
*                  - pBuffer : ָ��Ҫд��IIC���������ݵ�ָ�롣
*                  - WriteAddr : Ҫд���IIC�������ڲ���ַ��
*                  - NumByteToWrite: д����ֽڵĸ�����
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
* Description    : ��IIC�����ж����������ݡ�
* Input          : - DevAddr  :	Ҫд���IIC�����ĵ�ַ��
*                  - pBuffer : ָ���Ŵ�IIC�����������ݻ����ָ�롣
*                  - ReadAddr : ��IIC���������ݵ��ڲ���ַ��
*                  - NumByteToRead : Ҫ���������ݸ�����
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
* Description    : ��IIC�����ж����������ݡ�
* Input          : - DevAddr  :	Ҫд���IIC�����ĵ�ַ��
*                  - pBuffer : ָ���Ŵ�IIC�����������ݻ����ָ�롣
*                  - ReadAddr : ��IIC���������ݵ��ڲ���ַ��
*                  - NumByteToRead : Ҫ���������ݸ�����
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	I2C_GenerateSTART(I2C1, ENABLE);										   //������ʼλ
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  
  	//���ֻ��һ�����ݣ��ڶ�֮ǰ��ֹ��Ӧ�� 
  	if(NumByteToRead==1) 
  	{
    	I2C_AcknowledgeConfig(I2C1, DISABLE);
  	}
  
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);	          //����IIC������ַ��д��
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  	I2C_SendData(I2C1, ReadAddr);  											  //����IIC�ڲ���ַ��д��
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  	 
  	I2C_GenerateSTART(I2C1, ENABLE);										  //�ٴη�����ʼ���� 
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
  	
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);		          //����IIC������ַ������
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  	while(NumByteToRead)  
  	{												   	
    	if(I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    	{ 
      		if(NumByteToRead == 2)												  //�ڵ����ڶ����ֽ�ʱ��ֹӦ��
      		{       	
        		I2C_AcknowledgeConfig(I2C1, DISABLE);
      		}
      		if(NumByteToRead == 1)
      		{      	
        		I2C_GenerateSTOP(I2C1, ENABLE);									  //������ֽ�ʱ����ֹͣ����
      		}
      
      		*pBuffer = I2C_ReceiveData(I2C1);
      		pBuffer++; 
      		NumByteToRead--;    
    	}   
  	}
  	
  	I2C_AcknowledgeConfig(I2C1, ENABLE);										  //�ٴ�ʹ��Ӧ��׼����һ�ν���	
}

/*******************************************************************************
* Function Name  : void I2C_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
* Description    : ��IIC����д�������ֽڡ�
* Input          : - DevAddr  :	Ҫд���IIC�����ĵ�ַ��
*                  - pBuffer : ָ��Ҫд��IIC���������ݵ�ָ�롣
*                  - WriteAddr : Ҫд���IIC�������ڲ���ַ��
*                  - NumByteToWrite: д����ֽڵĸ�����
* Output         : None
* Return         : None 
*******************************************************************************/
void I2C_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite)
{
	I2C_GenerateSTART(I2C1, ENABLE);										 //������ʼλ
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
  
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);	         //����IIC������ַ��д��
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));  
 
  	I2C_SendData(I2C1, WriteAddr);  										 //����IIC�ڲ���ַ��д��
  	while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  	while(NumByteToWrite--)  
  	{  	
    	I2C_SendData(I2C1, *pBuffer); 									      //���͵�ǰ�ֽ�
    	pBuffer++; 
    	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  	}

  	I2C_GenerateSTOP(I2C1, ENABLE); 										  //����ֹͣλ 	
}
/*******************************************************************************
* Function Name  : void I2C_DEV_ByteWrite(uint8_t DevAddr, uint8_t pBuffer, uint8_t WriteAddr)
* Description    : дһ���ֽڵ�IIC�������ڲ��Ĵ�����
* Input          : - DevAddr  :	Ҫд���IIC�����ĵ�ַ��
*				   - pBuffer  : Ҫд������ݡ�
*				   - WriteAddr: Ҫд��ĵ�ַ��
* Output         : None
* Return         : None
*******************************************************************************/
void I2C_DEV_ByteWrite(uint8_t DevAddr, uint8_t pBuffer, uint8_t WriteAddr)
{
	//�ȵ�I2C���߲�æ
  	while(I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY));
    
	//������ʼλ
  	I2C_GenerateSTART(I2C1, ENABLE);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
  
   	//����IIC������������ַ��д��
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
   
    //����IIC�������ڲ���ַ��д��
  	I2C_SendData(I2C1, WriteAddr);  
  	while(! I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
   
	//��������
  	I2C_SendData(I2C1, pBuffer);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));  
   
   	//����ֹͣλ
  	I2C_GenerateSTOP(I2C1, ENABLE);
}
/*******************************************************************************
* Function Name  : uint8_t I2C_DEV_ByteRead(uint8_t DevAddr, uint8_t ReadAddr)
* Description    : ��IIC�����Ĵ��������ȡһ�����ݡ�
* Input          : - DevAddr  :	Ҫд���IIC�����ĵ�ַ��
*				   - ReadAddr : IIC�����ڲ��Ĵ����ĵ�ַ��
* Output         : None
* Return         : �����ļĴ���������
*******************************************************************************/
uint8_t I2C_DEV_ByteRead(uint8_t DevAddr, uint8_t ReadAddr)
{
	uint8_t RxData;
	//�ȵ�I2C���߲�æ
  	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	//������ʼλ
  	I2C_GenerateSTART(I2C1, ENABLE);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 

  	//����IIC������������ַ��д��
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Transmitter);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	//����IIC�������ڲ���ַ��д��
  	I2C_SendData(I2C1, ReadAddr);  
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
	//�ٴβ�����ʼλ 
  	I2C_GenerateSTART(I2C1, ENABLE);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
    
	//����IIC�������ڲ���ַ������
  	I2C_Send7bitAddress(I2C1, DevAddr, I2C_Direction_Receiver);
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); 

   	//�ȵ����ݽ��յ� 
  	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	//��������
	RxData = I2C_ReceiveData(I2C1);

	//��ֹӦ��
    I2C_AcknowledgeConfig(I2C1, DISABLE);

	//����ֹͣλ
    I2C_GenerateSTOP(I2C1, ENABLE);
	return RxData;
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
