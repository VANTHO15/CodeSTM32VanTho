#include "barometer.h"
#include "math.h"
// Đã được dịch sang trái 1 bit
#define MS5611_ADDRESS				  		(0xEE)

// Định nghĩa tập lệnh của module ms5611
#define MS5611_CMD_ADC_READ           		(0x00)
#define MS5611_CMD_RESET              		(0x1E)
#define MS5611_CMD_CONV_D1_4096           	(0x48)
#define MS5611_CMD_CONV_D2_4096            	(0x58)
#define MS5611_CMD_READ_PROM          		(0xA2)

Barometer::Barometer(I2C_HandleTypeDef * theI2c)
{
	this->hi2c = theI2c;
}
HAL_StatusTypeDef Barometer::Reset()
{
	uint8_t tempCmd = MS5611_CMD_RESET;
	if ( HAL_I2C_Master_Transmit(this->hi2c, (uint16_t) MS5611_ADDRESS, &tempCmd, 1, 500) != HAL_OK)
	{
		return HAL_ERROR;
	}
	HAL_Delay(10);
	return HAL_OK;
}

HAL_StatusTypeDef Barometer::ReadProm()
{
	uint8_t tempData[2] = {0};
	uint8_t theCmd = 0;
	for (uint8_t offset = 0; offset < 6; offset++)
	{
		theCmd = MS5611_CMD_READ_PROM + (offset * 2);
		if ( HAL_I2C_Master_Transmit(this->hi2c, (uint16_t) MS5611_ADDRESS, &theCmd, 1, 500) != HAL_OK){
			return HAL_ERROR;
		}
		HAL_Delay(10);
		if ( HAL_I2C_Master_Receive(this->hi2c, (uint16_t) MS5611_ADDRESS, tempData, 2, 500) != HAL_OK){
			return HAL_ERROR;
		}
		this->dataProm[offset] = tempData[0] << 8 | tempData[1];
	}
	return HAL_OK;
}
HAL_StatusTypeDef Barometer::Init()
{
	if ( Reset() != HAL_OK) {
		return HAL_ERROR;
	}

	if ( ReadProm() != HAL_OK){
		return HAL_ERROR;
	}

	return HAL_OK;
}
void Barometer::SetGroundPressure()
{
	this->actualPressure = 0;
}
uint32_t Barometer::GetDataFromPreviousRequest()
{
	uint8_t tempCmd = MS5611_CMD_ADC_READ;
	uint8_t tempData[3] = {0};

	// Đọc dữ liệu từ câu lệnh request trước đó
	HAL_I2C_Master_Transmit(this->hi2c, (uint16_t) MS5611_ADDRESS, &tempCmd, 1, 10);
	HAL_I2C_Master_Receive(this->hi2c, (uint16_t) MS5611_ADDRESS, tempData, 3, 10);
	return  ((tempData[0] << 16) | (tempData[1] << 8) | (tempData[2]));

}

BAROMETER_Result Barometer::IsReadyToInterface()
{
	/**** Kiểm tra giao tiếp I2C của MS5611 *****/
	if(HAL_I2C_IsDeviceReady(this->hi2c, MS5611_ADDRESS, 2, 5) != HAL_OK)
	{
		return BAROMETER_Result_DeviceNotConnected;
	}

	return BAROMETER_Result_Ok;
}

uint32_t Barometer::GetAverageTemperature()
{
	rawAverageTemperatureTotal -= rawTemperatureRotatingMemory[indexAverageTemperatureMem];
	// Đọc dữ liệu nhiệt độ
	rawTemperatureRotatingMemory[indexAverageTemperatureMem] = GetDataFromPreviousRequest();
	rawAverageTemperatureTotal += rawTemperatureRotatingMemory[indexAverageTemperatureMem];
	indexAverageTemperatureMem++;
	if (indexAverageTemperatureMem == 5)
	{
		indexAverageTemperatureMem = 0;
	}
	return rawAverageTemperatureTotal / 5;
}

void Barometer::RequestGetTemperatureData()
{
	uint8_t tempCmd = MS5611_CMD_CONV_D2_4096;
	HAL_I2C_Master_Transmit(this->hi2c, (uint16_t) MS5611_ADDRESS, &tempCmd, 1, 4);
}

void Barometer::RequestGetPressureData()
{
	uint8_t tempCmd = MS5611_CMD_CONV_D1_4096;
	HAL_I2C_Master_Transmit(this->hi2c, (uint16_t) MS5611_ADDRESS, &tempCmd, 1, 4);
}

int64_t Barometer::CompensatePressure()
{
	int32_t dT = rawTemperature - ((uint32_t)dataProm[4] << 8);
	int64_t OFF = (int64_t)dataProm[1] * 65536 + (int64_t)dataProm[3] * (int64_t)dT / 128;
	int64_t SENS = (int64_t)dataProm[0] * 32768 + (int64_t)dataProm[2] * dT / 256;
	int32_t TEMP;

	TEMP = 2000 + ((int64_t) dT * this->dataProm[5]) / 8388608;


	if (TEMP < -1500)
	{
		OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
		SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	else if (TEMP < 2000)
	{
		OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	else
	{
		OFF2 = 0;
		SENS2 = 0;
	}

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	return (((rawPressure * SENS) / 2097152 - OFF) / 32768);
}

float Barometer::GetAveragePressure()
{
	//To get a smoother pressure value we will use a 20 location rotating memory.
	averagePressureTotal -= pressureRotatingMemory[indexAveragePressureMem];
	pressureRotatingMemory[indexAveragePressureMem] = this->compensatedPressure;
	averagePressureTotal += pressureRotatingMemory[indexAveragePressureMem];
	indexAveragePressureMem++;
	if (indexAveragePressureMem == 20)
	{
		indexAveragePressureMem = 0;                              //Start at 0 when the memory location 20 is reached.
	}

	return (float)averagePressureTotal / 20.0;      //Calculate the average pressure of the last 20 pressure readings.
}

float Barometer::UseComplementaryFilter()
{
	//To get better results we will use a complementary fillter that can be adjusted by the fast average.
	actualPressureSlow = actualPressureSlow * (float)0.985 + actualPressureFast * (float)0.015;
	actualPressureDiff = actualPressureSlow - actualPressureFast;                                       //Calculate the difference between the fast and the slow avarage value.
	if (actualPressureDiff > 8)actualPressureDiff = 8;                                                    //If the difference is larger then 8 limit the difference to 8.
	if (actualPressureDiff < -8)actualPressureDiff = -8;                                                  //If the difference is smaller then -8 limit the difference to -8.
	//If the difference is larger then 1 or smaller then -1 the slow average is adjuste based on the error between the fast and slow average.
	if (actualPressureDiff > 1 || actualPressureDiff < -1)actualPressureSlow -= actualPressureDiff / 6.0;

	return actualPressureSlow;
}
void Barometer::CalculateLongtermChange()
{
	//In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
	//This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
	if (manualAltitudeChange == 1)
	{
		pressureParachutePrevious = actualPressure * 10;                       								//During manual altitude change the up/down detection is disabled.
	}
	parachuteThrottle -= parachuteBuffer[parachuteRotatingMemLocation];                                  	//Subtract the current memory position to make room for the new value.
	parachuteBuffer[parachuteRotatingMemLocation] = actualPressure * 10 - pressureParachutePrevious;   		//Calculate the new change between the actual pressure and the previous measurement.
	parachuteThrottle += parachuteBuffer[parachuteRotatingMemLocation];                                  	//Add the new value to the long term avarage value.
	pressureParachutePrevious = actualPressure * 10;                                                      	 //Store the current measurement for the next loop.
	parachuteRotatingMemLocation++;                                                                        	//Increase the rotating memory location.
	if (parachuteRotatingMemLocation == 30)
	{
		parachuteRotatingMemLocation = 0;                            //Start at 0 when the memory location 20 is reached.
	}
}

float Barometer::GetPidOutputAltitude()
{
	return this->pidOutputAltitude;
}
float Barometer::GetActualPressure()
{
	return this->actualPressure;
}

void Barometer::Set_PID_altitude_setpoint(float thePidAltitudeSetpoint)
{
	this->pidAltitudeSetpoint = thePidAltitudeSetpoint;
}

int16_t Barometer::GetManualThrottle()
{
	return this->manualThrottle;
}

void Barometer::CalculateAltitudePID(int16_t theThrottle)
{
	if (pidAltitudeSetpoint == 0)  pidAltitudeSetpoint = actualPressure;

	manualAltitudeChange = 0;
	manualThrottle = 0;
	if (theThrottle > 1600) {
		manualAltitudeChange = 1;
		pidAltitudeSetpoint = actualPressure;
		manualThrottle = (theThrottle - 1600) / 3;
	}
	if (theThrottle < 1400) {
		manualAltitudeChange = 1;
		pidAltitudeSetpoint = actualPressure;
		manualThrottle = (theThrottle - 1400) / 5;
	}

	pidAltitudeInput = actualPressure;
	pidErrorAltitudeTemp = pidAltitudeInput - pidAltitudeSetpoint;

	pidErrorGainAltitude = 0;
	if (pidErrorAltitudeTemp > 10 || pidErrorAltitudeTemp < -10)
	{
		pidErrorGainAltitude = (abs(pidErrorAltitudeTemp) - 10) / 20.0;
		if (pidErrorGainAltitude > 3)
		{
			pidErrorGainAltitude = 3;
		}
	}

	pid_Imem_Altitude += (pid_Igain_Altitude / 100.0) * pidErrorAltitudeTemp;
	if (pid_Imem_Altitude > pidMaxAltitude)
	{
		pid_Imem_Altitude = pidMaxAltitude;
	}
	else if (pid_Imem_Altitude < pidMaxAltitude * -1)
	{
		pid_Imem_Altitude = pidMaxAltitude * -1;
	}
	pidOutputAltitude = (pid_Pgain_Altitude + pidErrorGainAltitude) * pidErrorAltitudeTemp + pid_Imem_Altitude + pid_Dgain_Altitude * parachuteThrottle;
	if (pidOutputAltitude > pidMaxAltitude)
	{
		pidOutputAltitude = pidMaxAltitude;
	}
	else if (pidOutputAltitude < pidMaxAltitude * -1)
	{
		pidOutputAltitude = pidMaxAltitude * -1;
	}
}

void Barometer::ResetValuesOfPID()
{
	pidAltitudeSetpoint = 0;
	pidOutputAltitude = 0;
	pid_Imem_Altitude = 0;
	manualThrottle = 0;
	manualAltitudeChange = 1;
}

void Barometer::SetKgainPID(float thePgain,float theIgain, float theDgain, int16_t theMaxPID)
{
	this->pid_Pgain_Altitude = thePgain;
	this->pid_Igain_Altitude = theIgain;
	this->pid_Dgain_Altitude = theDgain;
	this->pidMaxAltitude = theMaxPID;
}
void Barometer::ReadAndCalculatePIDBarometer(uint8_t theFlightMode, uint8_t theTakeoffDetected, uint16_t throttle )
{
	stageOfBarometer++;

	if (stageOfBarometer == 1)
	{
		if (stageOfTemperature == 0)
		{
			rawTemperature = GetAverageTemperature();
		}
		else
		{
			rawPressure = GetDataFromPreviousRequest();
		}

		stageOfTemperature++;

		if(stageOfTemperature == 20)
		{
			stageOfTemperature = 0;
			RequestGetTemperatureData();
		}
		else
		{
			RequestGetPressureData();
		}
	}

	if (stageOfBarometer == 2)
	{
		compensatedPressure = CompensatePressure();
		actualPressureFast = GetAveragePressure();
		actualPressure = UseComplementaryFilter();
	}

	if (stageOfBarometer == 3)
	{
		stageOfBarometer = 0;
		CalculateLongtermChange();
		if (theFlightMode >= 2 && theTakeoffDetected ==1)
		{
			CalculateAltitudePID(throttle);
		}

		else if (theFlightMode < 2 && pidAltitudeSetpoint != 0)
		{
			ResetValuesOfPID();
		}
	}
}
