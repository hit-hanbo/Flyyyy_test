#include "MPU6050.h"


void HiSTM_MPU6050_Send_Data(uint8_t register_address, uint8_t data)
{
	uint8_t buffer[2] = { 0, 0 };
	buffer[0] = register_address; buffer[1] = data;
	HAL_I2C_Master_Transmit(MPU6050_I2C_INTERFACE, MPU6050_ADDRESS, buffer, 2, 0xFF);
}

uint8_t HiSTM_MPU6050_Recv_Data(uint8_t register_address)
{
	uint8_t temp;
	HAL_I2C_Master_Transmit(MPU6050_I2C_INTERFACE, MPU6050_ADDRESS, &register_address, 1, 0xFF);
	HAL_I2C_Master_Receive(MPU6050_I2C_INTERFACE, MPU6050_ADDRESS, &temp, 1, 0xFF);
	return temp;
}

uint8_t HiSTM_MPU6050_whoAmI(void)
{
	uint8_t temp;
	temp = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_WHOAMI);
	return temp;
}

uint8_t HiSTM_MPU6050_Init(void)
{
	volatile uint8_t temp = 0;
	//  Start to reset IMU
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_PWR_MGMT_1, 0x80);
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_PWR_MGMT_1, 0x80);
	//  Wait for IMU reset
	HiSTM_Delay(50);
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_PWR_MGMT_1, 0x03);
	temp = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_PWR_MGMT_1);
	//  Clk source PLLx
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_PWR_MGMT_1, 0x01);
	temp = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_PWR_MGMT_1);


	//  Check if IMU exists
	temp = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_WHOAMI);
	if(  temp != 0x68)
		return MPU6050_ERROR_NO_DEV;
	//  Disable test mode, Set Gyro Full Scale +-2000dps
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_GYRO_CONFIG, 0x18);
	//  Disable test modem Set ACCEL Full Scale +-2g
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_ACCEL_CONFIG, 0x00);
	//  Disable IMU Interrupt Output
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_INT_ENABLE, 0x00);
	//  Disable IMU Aux I2C
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_INT_PIN_CFG, 0x00);
	//  Set Sample rate 500Hz
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_SMPLRT_DIV, 1);
	//  Set Filter
	HiSTM_MPU6050_Send_Data(MPU6050_REGISTER_CONFIG, 2);

	return MPU6050_DEV_OK;
}

MPU6050_RawDataStructTypeDef HiSTM_MPU6050_Get_RawData(void)
{
	MPU6050_RawDataStructTypeDef temp_raw_data;
	uint8_t temp_value_msb, temp_value_lsb;
	temp_value_lsb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_GYRO_XOUT_L);
	temp_value_msb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_GYRO_XOUT_H);
	temp_raw_data.Gyro_X = (int16_t) ((temp_value_msb << 8 ) | (temp_value_lsb));

	temp_value_lsb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_GYRO_YOUT_L);
	temp_value_msb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_GYRO_YOUT_H);
	temp_raw_data.Gyro_Y = (int16_t) ((temp_value_msb << 8 ) | (temp_value_lsb));

	temp_value_lsb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_GYRO_ZOUT_L);
	temp_value_msb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_GYRO_ZOUT_H);
	temp_raw_data.Gyro_Z = (int16_t) ((temp_value_msb << 8 ) | (temp_value_lsb));

	temp_value_lsb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_ACCEL_XOUT_L);
	temp_value_msb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_ACCEL_XOUT_H);
	temp_raw_data.Accel_X = (int16_t) ((temp_value_msb << 8 ) | (temp_value_lsb));

	temp_value_lsb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_ACCEL_YOUT_L);
	temp_value_msb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_ACCEL_YOUT_H);
	temp_raw_data.Accel_Y = (int16_t) ((temp_value_msb << 8 ) | (temp_value_lsb));

	temp_value_lsb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_ACCEL_ZOUT_L);
	temp_value_msb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_ACCEL_ZOUT_H);
	temp_raw_data.Accel_Z = (int16_t) ((temp_value_msb << 8 ) | (temp_value_lsb));

	temp_value_lsb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_TEMP_OUT_L);
	temp_value_msb = HiSTM_MPU6050_Recv_Data(MPU6050_REGISTER_TEMP_OUT_H);
	temp_raw_data.Temperature = (int16_t) ((temp_value_msb << 8 ) | (temp_value_lsb));

	return temp_raw_data;
}

void HiSTM_MPU6050_Kalman_Filter()
{

}


