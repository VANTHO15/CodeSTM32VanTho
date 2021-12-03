/*
 * GY86.c
 *
 *  Created on: May 30, 2021
 *      Author: Admin
 */

#include "stm32f4xx_hal.h"
#include "GY86.h"

int Accel_X_RAW,Accel_Y_RAW,Accel_Z_RAW,Gyro_X_RAW,Gyro_Y_RAW,Gyro_Z_RAW;
float  Ax,Ay,Az,Gx,Gy,Gz;
extern I2C_HandleTypeDef hi2c1;

//IMU9DOF_Result MPU6050_Init()
//{
//	uint8_t check=0,Data;
//
//	/**** Kiểm tra giao tiếp I2C của MPU6050 *****/
//
//	if (HAL_I2C_IsDeviceReady(&hi2c1,MPU6050_ADDR, 2, 5) != HAL_OK)
//	{
//		return IMU10DOF_Result_MPU6050_NotConnected_I2C;
//	}
//
//	/* Check who am I */
//	if(HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000)!=HAL_OK)
//	{
//		return IMU10DOF_Result_MPU6050_NotConnected_RESPON;
//	}
//	/* Checking */
//	while(check != 104)
//	{
//		/* Return error */
//		return IMU10DOF_Result_MPU6050_NotConnected;
//	}
//	/* Wakeup MPU6050 */
//	Data = 0;
//	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000) != HAL_OK)
//	{
//		return IMU10DOF_Result_MPU6050_NotConnected_WAKEUP;
//	}
//
//	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
//	Data = 0x07;
//	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
//	// set accelerometer configuration in ACCEL_CONGIG Resister
//	// XA_ST=0, YA_ST=0,ZA_ST=0,FS_SEL=0 ->+-2g
//	Data=0x00;
//	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
//
//	return IMU10DOF_Result_Ok;
//}
//
//void MPU6050_Read_Accel()
//{
//	uint8_t Rec_Data[6];
//	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
//
//	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
//
//	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
//	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
//	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
//
//	Ax = Accel_X_RAW/16384.0;  // get the float g
//	Ay = Accel_Y_RAW/16384.0;
//	Az = Accel_Z_RAW/16384.0;
//
//}
//void MPU6050_Read_Gyro()
//{
//	uint8_t Rec_Data[6];
//	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
//
//	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
//
//	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
//	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
//	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
//
//	Gx=Gyro_X_RAW/131.0;
//	Gy=Gyro_Y_RAW/131.0;
//	Gz=Gyro_Z_RAW/131.0;
//
//}
