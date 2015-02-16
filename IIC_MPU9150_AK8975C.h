/*
******************************************************************
* 文件：IIC_MPU9150_AK8975C.H
* 编辑：合嵌科技
* 日期：2013/12/16
* 修改：
* 版本：V1.0
* 描述：IIC MPU9150三轴磁传感器操作函数的头文件。
******************************************************************
*/
/* ----------防止重定义---------------------------*/
#ifndef __IIC_MPU9150_AK8975C_H
#define __IIC_MPU9150_AK8975C_H

/* Includes --------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

#define MPU9150_AK8975C_ADDR    0x18 //AK8975C器件地址	0000 1100<<1 ---> 0001 1000

/* ----------寄存器宏定义-------------------------*/
/* ----------Read Only Register-------------------*/
#define MPU9150_AK8975C_WIA     0x00 //RO	Device ID
#define MPU9150_AK8975C_INFO    0x01 //RO	information 
#define MPU9150_AK8975C_ST1     0x02 //RO	Status 1---data status

#define MPU9150_AK8975C_HXL     0x03 //RO	X measurement data lower 8-bit
#define MPU9150_AK8975C_HXH     0x04 //RO	X measurement data higher 8-bit
#define MPU9150_AK8975C_HYL     0x05 //RO	Y measurement data lower 8-bit
#define MPU9150_AK8975C_HYH     0x06 //RO	Y measurement data higher 8-bit
#define MPU9150_AK8975C_HZL     0x07 //RO	Z measurement data lower 8-bit
#define MPU9150_AK8975C_HZH     0x08 //RO	Z measurement data higher 8-bit

#define MPU9150_AK8975C_ST2     0x09 //RO	Status 2

#define MPU9150_AK8975C_ASAX    0x10 //RO 	Magnetic sensor X-axis sensitivity adjustment value
#define MPU9150_AK8975C_ASAY    0x11 //RO 	Magnetic sensor Y-axis sensitivity adjustment value
#define MPU9150_AK8975C_ASAZ    0x12 //RO 	Magnetic sensor Z-axis sensitivity adjustment value
/* ----------Read/Write Register-------------------*/
#define MPU9150_AK8975C_CNTL    0x0A //RW  Operation mode setting 
#define MPU9150_AK8975C_RSV	    0x0B //RW	Reserved
#define MPU9150_AK8975C_ASTC	0x0C //RW	Self Test Control
#define MPU9150_AK8975C_TS1	    0x0D //RW	Test register for shipment.DO NOT USE these registers.
#define MPU9150_AK8975C_TS2	    0x0E //RW	Test register for shipment.DO NOT USE these registers.
#define MPU9150_AK8975C_I2CDIS	0x0F //RW	Disable I2C bus interface.

/*功能宏定义---------------------------------------*/
#define _AKM_cMode_PwrDown      0x00 //Power-down mode
#define _AKM_cMode_SglMeas      0x01 //Single measurement mode
#define _AKM_cMode_SelfTest     0x08 //Self-test mode
#define _AKM_cMode_FuseROMacc   0x0F //Fuse ROM access mode 用于读取0x10-0x12地址的数据
#define _AKM_ST_EN              0x40
#define _AKM_ST_DIS             0x00
#define _AKM_DRDY              	0x01 //data is ready
/*AK8975C function---------------------------------*/
void AK8975C_SetMode(uint8_t mode);
void AK8975C_Init(void);
uint8_t AK8975C_SelfTest(void);
void AK8975C_GetMagnet(float* mx, float* my, float* mz, int16_t* i_mx, int16_t* i_my, int16_t* i_mz);

#endif   /* __IIC_MPU9150_AK8975C_H*/

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
