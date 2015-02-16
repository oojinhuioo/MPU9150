/*
******************************************************************
* 文件：IIC_MPU6050.h
* 编辑：合嵌科技
* 日期：2012/12/2
* 修改：
* 版本：V1.0
* 描述：IIC MPU6050三轴加速度、三轴陀螺仪传感器头文件。
******************************************************************
*/
/* ----------防止重定义---------------------------*/
#ifndef __IIC_MPU6050_H
#define __IIC_MPU6050_H

/* Includes --------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>


#define MPU6050_ADDR             0xD0 //MPU6050 IIC器件地址
#define MPU6050_ADDR_AD0_LOW     0x68 // AD0 -- GND , 从WHO_AM_I读出
#define MPU6050_ADDR_AD0_HIGH    0x69 // AD0 -- VCC ，从WHO_AM_I读出
#define MPU6050_ADDR_DEFAULT     MPU6050_ADDR_AD0_LOW

/*MPU6050 寄存器地址描述---------------------------结合了寄存器映射和描述手册V3.2和V4.0，两者寄存器描述有互有缺少*/
//自测寄存器
#define MPU6050_SELF_TEST_X     0x0D	//RW  XA_TEST[4：2]   XG_TEST[4:0]
#define MPU6050_SELF_TEST_Y     0x0E    //RW  YA_TEST[4：2]   YG_TEST[4:0]
#define MPU6050_SELF_TEST_Z     0x0F	//RW  ZA_TEST[4：2]   ZG_TEST[4:0]
#define MPU6050_SELF_TEST_A     0x10 	//RW  XA_TEST[1：0]   YA_TEST[1：0]	  ZA_TEST[1：0]

#define MPU6050_SMPLRT_DIV      0x19    //采样率分频
#define MPU6050_CONFIG          0x1A	//配置寄存器  外部帧同步FSYNC 数字低通滤波器DLFP
#define MPU6050_GYRO_CONFIG     0x1B    //陀螺仪配置  各轴自测使能 	量程选择FS_SEL
#define MPU6050_ACCEL_CONFIG    0x1C	//加速度计配置 各轴自测使能 	量程选择AFS_SEL	DHPF
#define MPU6050_FF_THR          0x1D	//自由落体加速度阀值
#define MPU6050_FF_DUR          0x1E	//自由落体时间
#define MPU6050_MOT_THR        	0x1F    //运动检测阀值
#define MPU6050_MOT_DUR        	0x1F	//运动检测时间
#define MPU6050_ZRMOT_THR       0x21	//零运动检测阀值
#define MPU6050_ZRMOT_DUR       0x22	//零运动检测时间
#define MPU6050_FIFO_EN			0x23	//FIFO使能
//辅I2C总线寄存器
#define MPU6050_I2C_MST_CTRL	0x24	//I2C主控制
#define MPU6050_I2C_SLV0_ADDR	0x25	//I2C从器件0地址
#define MPU6050_I2C_SLV0_REG    0x26    //I2C从器件0内部起始寄存器
#define MPU6050_I2C_SLV0_CTRL   0x27	//I2C从器件0控制
#define MPU6050_I2C_SLV1_ADDR	0x28	//I2C从器件1地址
#define MPU6050_I2C_SLV1_REG    0x29    //I2C从器件1内部起始寄存器
#define MPU6050_I2C_SLV1_CTRL   0x2A	//I2C从器件1控制
#define MPU6050_I2C_SLV2_ADDR	0x2B	//I2C从器件2地址
#define MPU6050_I2C_SLV2_REG    0x2C    //I2C从器件2内部起始寄存器
#define MPU6050_I2C_SLV2_CTRL   0x2D	//I2C从器件2控制
#define MPU6050_I2C_SLV3_ADDR	0x2E	//I2C从器件3地址
#define MPU6050_I2C_SLV3_REG    0x2F    //I2C从器件3内部起始寄存器
#define MPU6050_I2C_SLV3_CTRL   0x30	//I2C从器件3控制
#define MPU6050_I2C_SLV4_ADDR	0x31	//I2C从器件4地址
#define MPU6050_I2C_SLV4_REG    0x32    //I2C从器件4内部起始寄存器
#define MPU6050_I2C_SLV4_DO     0x33	//I2C从器件4数据输出
#define MPU6050_I2C_SLV4_CTRL   0x34	//I2C从器件4控制
#define MPU6050_I2C_SLV4_DI     0x35	//I2C从器件4数据输入
#define MPU6050_I2C_MST_STATUS  0x36	//I2C主状态	  RO  
//中断寄存器
#define MPU6050_INT_PIN_CFG		0x37    //INT引脚/旁路使能配置 
#define MPU6050_INT_ENABLE      0x38	//中断使能
#define MPU6050_INT_STATUS      0x3A	//中断状态
//加速度测量寄存器
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
//温度测量寄存器
#define MPU6050_TEMP_OUT_H      0x41     
#define MPU6050_TEMP_OUT_L      0x42
//陀螺仪测量寄存器
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
//外部传感器数据0-23   RO
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
#define MPU6050_MOT_DETECT_STATUS  0x61  //运动检测状态寄存器
//从0 1 2 3数据输出寄存器
#define MPU6050_I2C_SLV0_DO        0x63
#define MPU6050_I2C_SLV1_DO        0x64
#define MPU6050_I2C_SLV2_DO        0x65
#define MPU6050_I2C_SLV3_DO        0x66
#define MPU6050_I2C_MST_DELAY_CTRL 0x67	  //I2C主延迟控制寄存器
#define MPU6050_SIGNAL_PATH_RESET  0x68   //信号路径复位
#define MPU6050_MOT_DETECT_CTRL    0x69	  //运动检测控制
#define MPU6050_USER_CTRL          0x6A	  //用户控制
#define MPU6050_PWR_MGMT_1         0x6B	  //电源管理1 
#define MPU6050_PWR_MGMT_2         0x6C	  //电源管理2
#define MPU6050_FIFO_COUNTH        0x72	  //FIFO计数寄存器H	   RO
#define MPU6050_FIFO_COUNTL        0x73	  //FIFO计数寄存器L	   RO
#define MPU6050_FIFO_R_W           0x74	  //FIFO读写寄存器
#define MPU6050_WHO_AM_I           0x75	  //存器件的I2C地址    RO

/*功能宏定义---------------------------*/
//数字低通滤波器（以陀螺仪做参考） 配置宏
#define _CONFIG_DLPF_BW_256        0x00 
#define _CONFIG_DLPF_BW_188        0x01
#define _CONFIG_DLPF_BW_98         0x02 
#define _CONFIG_DLPF_BW_42         0x03
#define _CONFIG_DLPF_BW_20         0x04 
#define _CONFIG_DLPF_BW_10         0x05
#define _CONFIG_DLPF_BW_5          0x06
//陀螺仪量程 配置宏
#define _gCONFIG_FS_250	          0x00 
#define _gCONFIG_FS_500	          (0x01<<3) 
#define _gCONFIG_FS_1000          (0x02<<3) 
#define _gCONFIG_FS_2000          (0x03<<3) 
#define _gCONFIG_XYZG_ST		  0xE0
#define _gCONFIG_XG_ST		      0x80
#define _gCONFIG_YG_ST		      0x40
#define _gCONFIG_ZG_ST		      0x20
//加速度计量程、数字高通滤波器 配置宏
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
//电源管理 时钟和睡眠
#define _CLOCK_INTERNAL            0x00	 //内部8M晶振
#define _CLOCK_PLL_XGYRO           0x01	 //带有X轴陀螺仪参考的PLL
#define _CLOCK_PLL_YGYRO           0x02	 //带有Y轴陀螺仪参考的PLL
#define _CLOCK_PLL_ZGYRO           0x03	 //带有Z轴陀螺仪参考的PLL
#define _CLOCK_PLL_EXT32K          0x04	 //带有外部32.768KHz参考的PLL
#define _CLOCK_PLL_EXT19M          0x05	 //带有外部19.2MHz参考的PLL
#define _CLOCK_KEEP_RESET          0x07	 //停止时钟
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
