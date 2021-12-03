/*
 * GY86.h
 *
 *  Created on: May 30, 2021
 *      Author: Admin
 */

#ifndef INC_GY86_H_
#define INC_GY86_H_

typedef enum  {
	IMU10DOF_Result_Ok = 0x00,
	IMU10DOF_Result_DeviceNotConnected, //
	IMU10DOF_Result_Error,
	IMU10DOF_Result_DeviceInvalid,       // Không phải địa chỉ của mpu6050
	IMU10DOF_Result_MPU6050_NotConnected,
	IMU10DOF_Result_MPU6050_NotConnected_I2C,
	IMU10DOF_Result_MPU6050_NotConnected_RESPON,
	IMU10DOF_Result_MPU6050_NotConnected_WAKEUP,
	IMU10DOF_Result_HMC5883L_NotConnected
} IMU9DOF_Result;


#define MPU6050_ADDR     0xD0
#define HMC5883L_ADDRESS 0b0011110
#define MS5611_ADDRESS   0b1110111
#define SMPLRT_DIV_REG    0x19
#define GYRO_CONFIG_REG    0x1B
#define ACCEL_CONFIG_REG    0x1C
#define ACCEL_XOUT_H_REG    0x3B
#define TEMP_OUT_H_REG    0x41
#define GYRO_XOUT_H_REG    0x43
#define PWR_MGMT_1_REG    0x6B
#define WHO_AM_I_REG    0x75

//IMU9DOF_Result MPU6050_Init();
//void MPU6050_Read_Accel();
//void MPU6050_Read_Gyro();




#endif /* INC_GY86_H_ */
