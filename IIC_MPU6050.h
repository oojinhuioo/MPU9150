/*
******************************************************************
* �ļ���IIC_MPU6050.h
* �༭����Ƕ�Ƽ�
* ���ڣ�2012/12/2
* �޸ģ�
* �汾��V1.0
* ������IIC MPU6050������ٶȡ����������Ǵ�����ͷ�ļ���
******************************************************************
*/
/* ----------��ֹ�ض���---------------------------*/
#ifndef __IIC_MPU6050_H
#define __IIC_MPU6050_H

/* Includes --------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>


#define MPU6050_ADDR             0xD0 //MPU6050 IIC������ַ
#define MPU6050_ADDR_AD0_LOW     0x68 // AD0 -- GND , ��WHO_AM_I����
#define MPU6050_ADDR_AD0_HIGH    0x69 // AD0 -- VCC ����WHO_AM_I����
#define MPU6050_ADDR_DEFAULT     MPU6050_ADDR_AD0_LOW

/*MPU6050 �Ĵ�����ַ����---------------------------����˼Ĵ���ӳ��������ֲ�V3.2��V4.0�����߼Ĵ��������л���ȱ��*/
//�Բ�Ĵ���
#define MPU6050_SELF_TEST_X     0x0D	//RW  XA_TEST[4��2]   XG_TEST[4:0]
#define MPU6050_SELF_TEST_Y     0x0E    //RW  YA_TEST[4��2]   YG_TEST[4:0]
#define MPU6050_SELF_TEST_Z     0x0F	//RW  ZA_TEST[4��2]   ZG_TEST[4:0]
#define MPU6050_SELF_TEST_A     0x10 	//RW  XA_TEST[1��0]   YA_TEST[1��0]	  ZA_TEST[1��0]

#define MPU6050_SMPLRT_DIV      0x19    //�����ʷ�Ƶ
#define MPU6050_CONFIG          0x1A	//���üĴ���  �ⲿ֡ͬ��FSYNC ���ֵ�ͨ�˲���DLFP
#define MPU6050_GYRO_CONFIG     0x1B    //����������  �����Բ�ʹ�� 	����ѡ��FS_SEL
#define MPU6050_ACCEL_CONFIG    0x1C	//���ٶȼ����� �����Բ�ʹ�� 	����ѡ��AFS_SEL	DHPF
#define MPU6050_FF_THR          0x1D	//����������ٶȷ�ֵ
#define MPU6050_FF_DUR          0x1E	//��������ʱ��
#define MPU6050_MOT_THR        	0x1F    //�˶���ֵⷧ
#define MPU6050_MOT_DUR        	0x1F	//�˶����ʱ��
#define MPU6050_ZRMOT_THR       0x21	//���˶���ֵⷧ
#define MPU6050_ZRMOT_DUR       0x22	//���˶����ʱ��
#define MPU6050_FIFO_EN			0x23	//FIFOʹ��
//��I2C���߼Ĵ���
#define MPU6050_I2C_MST_CTRL	0x24	//I2C������
#define MPU6050_I2C_SLV0_ADDR	0x25	//I2C������0��ַ
#define MPU6050_I2C_SLV0_REG    0x26    //I2C������0�ڲ���ʼ�Ĵ���
#define MPU6050_I2C_SLV0_CTRL   0x27	//I2C������0����
#define MPU6050_I2C_SLV1_ADDR	0x28	//I2C������1��ַ
#define MPU6050_I2C_SLV1_REG    0x29    //I2C������1�ڲ���ʼ�Ĵ���
#define MPU6050_I2C_SLV1_CTRL   0x2A	//I2C������1����
#define MPU6050_I2C_SLV2_ADDR	0x2B	//I2C������2��ַ
#define MPU6050_I2C_SLV2_REG    0x2C    //I2C������2�ڲ���ʼ�Ĵ���
#define MPU6050_I2C_SLV2_CTRL   0x2D	//I2C������2����
#define MPU6050_I2C_SLV3_ADDR	0x2E	//I2C������3��ַ
#define MPU6050_I2C_SLV3_REG    0x2F    //I2C������3�ڲ���ʼ�Ĵ���
#define MPU6050_I2C_SLV3_CTRL   0x30	//I2C������3����
#define MPU6050_I2C_SLV4_ADDR	0x31	//I2C������4��ַ
#define MPU6050_I2C_SLV4_REG    0x32    //I2C������4�ڲ���ʼ�Ĵ���
#define MPU6050_I2C_SLV4_DO     0x33	//I2C������4�������
#define MPU6050_I2C_SLV4_CTRL   0x34	//I2C������4����
#define MPU6050_I2C_SLV4_DI     0x35	//I2C������4��������
#define MPU6050_I2C_MST_STATUS  0x36	//I2C��״̬	  RO  
//�жϼĴ���
#define MPU6050_INT_PIN_CFG		0x37    //INT����/��·ʹ������ 
#define MPU6050_INT_ENABLE      0x38	//�ж�ʹ��
#define MPU6050_INT_STATUS      0x3A	//�ж�״̬
//���ٶȲ����Ĵ���
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
//�¶Ȳ����Ĵ���
#define MPU6050_TEMP_OUT_H      0x41     
#define MPU6050_TEMP_OUT_L      0x42
//�����ǲ����Ĵ���
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
//�ⲿ����������0-23   RO
#define MPU6050_EXT_SENS_DATA_00 0x49
#define MPU6050_EXT_SENS_DATA_01 0x4A
#define MPU6050_EXT_SENS_DATA_02 0x4B
#define MPU6050_EXT_SENS_DATA_03 0x4C
#define MPU6050_EXT_SENS_DATA_04 0x4D
#define MPU6050_EXT_SENS_DATA_05 0x4E
#define MPU6050_EXT_SENS_DATA_06 0x4F
#define MPU6050_EXT_SENS_DATA_07 0x50
#define MPU6050_EXT_SENS_DATA_08 0x51
#define MPU6050_EXT_SENS_DATA_09 0x52
#define MPU6050_EXT_SENS_DATA_10 0x53
#define MPU6050_EXT_SENS_DATA_11 0x54
#define MPU6050_EXT_SENS_DATA_12 0x55
#define MPU6050_EXT_SENS_DATA_13 0x56
#define MPU6050_EXT_SENS_DATA_14 0x57
#define MPU6050_EXT_SENS_DATA_15 0x58
#define MPU6050_EXT_SENS_DATA_16 0x59
#define MPU6050_EXT_SENS_DATA_17 0x5A
#define MPU6050_EXT_SENS_DATA_18 0x5B
#define MPU6050_EXT_SENS_DATA_19 0x5C
#define MPU6050_EXT_SENS_DATA_20 0x5D
#define MPU6050_EXT_SENS_DATA_21 0x5E
#define MPU6050_EXT_SENS_DATA_22 0x5F
#define MPU6050_EXT_SENS_DATA_23 0x60
#define MPU6050_MOT_DETECT_STATUS  0x61  //�˶����״̬�Ĵ���
//��0 1 2 3��������Ĵ���
#define MPU6050_I2C_SLV0_DO        0x63
#define MPU6050_I2C_SLV1_DO        0x64
#define MPU6050_I2C_SLV2_DO        0x65
#define MPU6050_I2C_SLV3_DO        0x66
#define MPU6050_I2C_MST_DELAY_CTRL 0x67	  //I2C���ӳٿ��ƼĴ���
#define MPU6050_SIGNAL_PATH_RESET  0x68   //�ź�·����λ
#define MPU6050_MOT_DETECT_CTRL    0x69	  //�˶�������
#define MPU6050_USER_CTRL          0x6A	  //�û�����
#define MPU6050_PWR_MGMT_1         0x6B	  //��Դ����1 
#define MPU6050_PWR_MGMT_2         0x6C	  //��Դ����2
#define MPU6050_FIFO_COUNTH        0x72	  //FIFO�����Ĵ���H	   RO
#define MPU6050_FIFO_COUNTL        0x73	  //FIFO�����Ĵ���L	   RO
#define MPU6050_FIFO_R_W           0x74	  //FIFO��д�Ĵ���
#define MPU6050_WHO_AM_I           0x75	  //��������I2C��ַ    RO

/*���ܺ궨��---------------------------*/
//���ֵ�ͨ�˲����������������ο��� ���ú�
#define _CONFIG_DLPF_BW_256        0x00 
#define _CONFIG_DLPF_BW_188        0x01
#define _CONFIG_DLPF_BW_98         0x02 
#define _CONFIG_DLPF_BW_42         0x03
#define _CONFIG_DLPF_BW_20         0x04 
#define _CONFIG_DLPF_BW_10         0x05
#define _CONFIG_DLPF_BW_5          0x06
//���������� ���ú�
#define _gCONFIG_FS_250	          0x00 
#define _gCONFIG_FS_500	          (0x01<<3) 
#define _gCONFIG_FS_1000          (0x02<<3) 
#define _gCONFIG_FS_2000          (0x03<<3) 
#define _gCONFIG_XYZG_ST		  0xE0
#define _gCONFIG_XG_ST		      0x80
#define _gCONFIG_YG_ST		      0x40
#define _gCONFIG_ZG_ST		      0x20
//���ٶȼ����̡����ָ�ͨ�˲��� ���ú�
#define _aCONFIG_FS_2			   0x00
#define _aCONFIG_FS_4			   (0x01<<3) 
#define _aCONFIG_FS_8			   (0x02<<3) 
#define _aCONFIG_FS_16			   (0x03<<3)
#define _aCONFIG_DHPF_RESET        0x00 
#define _aCONFIG_DHPF_5            0x01
#define _aCONFIG_DHPF_2P5          0x02
#define _aCONFIG_DHPF_1P25         0x03
#define _aCONFIG_DHPF_0P63         0x04
#define _aCONFIG_DHPF_HOLD         0x06
#define _aCONFIG_XYZA_ST     	   0xE0
#define _aCONFIG_XA_ST     	       0x80
#define _aCONFIG_YA_ST     	       0x40
#define _aCONFIG_ZA_ST     	       0x20
//��Դ���� ʱ�Ӻ�˯��
#define _CLOCK_INTERNAL            0x00	 //�ڲ�8M����
#define _CLOCK_PLL_XGYRO           0x01	 //����X�������ǲο���PLL
#define _CLOCK_PLL_YGYRO           0x02	 //����Y�������ǲο���PLL
#define _CLOCK_PLL_ZGYRO           0x03	 //����Z�������ǲο���PLL
#define _CLOCK_PLL_EXT32K          0x04	 //�����ⲿ32.768KHz�ο���PLL
#define _CLOCK_PLL_EXT19M          0x05	 //�����ⲿ19.2MHz�ο���PLL
#define _CLOCK_KEEP_RESET          0x07	 //ֹͣʱ��
#define _PWR_RUN				   0x00
#define _PWR_SLEEP_EN			   0x40
#define _PWR_CYCLE_EN			   0x20

/* Exported functions -------------------------------*/
void I2C_DEV_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void I2C_DEV_BufferWrite(uint8_t DevAddr, uint8_t* pBuffer, uint8_t WriteAddr, uint8_t NumByteToWrite);
void I2C_BufferRead(uint8_t DevAddr, uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
uint8_t I2C_DEV_ByteRead(uint8_t DevAddr, uint8_t ReadAddr);
void I2C_DEV_ByteWrite(uint8_t DevAddr, uint8_t pBuffer, uint8_t WriteAddr);

void MPU6050_Init(void);
uint8_t MPU6050_SelfTest(void);
void MPU6050_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
float MPU6050_GetTemperature(void);

void delay_msec(int loop_times);

#endif   /* __IIC_BMP180_H__ */
