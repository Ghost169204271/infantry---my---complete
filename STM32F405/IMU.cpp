#include"IMU.h"
bool IMU::crc_test(uint8_t* date_IMU)
{
	uint16_t true_crc=(date_IMU[5]<<8)+date_IMU[4];
	uint16_t now_crc = 0;
	crc16_update(&now_crc, date_IMU, 4);
	uint32_t length = (date_IMU[3] << 8) + date_IMU[2];
	crc16_update(&now_crc, date_IMU + 6, length);
	return true_crc == now_crc;
}
void IMU::Init()
{
	//this->initialYaw = this->angle.yaw;
	state = 1;
}
uint8_t* IMU::GetDMARx()
{
	return rxData;
}
void IMU::OnIRQHandler(size_t rxSize)
{
	if ((crc_test(rxData))&&( rxSize==MAX_DateLength)&&state)
	{
		
		this->angle.pitch = Get_date(rxData[_PAYLOAD_EulerAngles_ + 1], rxData[_PAYLOAD_EulerAngles_ + 2])/100.f;
		this->angle.roll = Get_date(rxData[_PAYLOAD_EulerAngles_ + 3], rxData[_PAYLOAD_EulerAngles_ + 4]) / 100.f;
		this->angle.yaw = Get_date(rxData[_PAYLOAD_EulerAngles_ + 5], rxData[_PAYLOAD_EulerAngles_ + 6])/10.f;

		imuKalman[YAW].Filter(this->angle.yaw);

		this->angularVelocity.pitch = Get_date(rxData[_PAYLOAD_AngularVelocity_ + 1], rxData[_PAYLOAD_AngularVelocity_ + 2])/10.f;
		this->angularVelocity.roll = Get_date(rxData[_PAYLOAD_AngularVelocity_ + 3], rxData[_PAYLOAD_AngularVelocity_ + 4])/10.f;
		this->angularVelocity.yaw = Get_date(rxData[_PAYLOAD_AngularVelocity_ + 5], rxData[_PAYLOAD_AngularVelocity_ + 6])/10.f;
	
		this->acceleration.pitch = Get_date(rxData[_PAYLOAD_Acceleration_ + 1], rxData[_PAYLOAD_Acceleration_ + 2]) / 1000.f;
		this->acceleration.roll = Get_date(rxData[_PAYLOAD_Acceleration_ + 3], rxData[_PAYLOAD_Acceleration_ + 4]) / 1000.f;
		this->acceleration.yaw = Get_date(rxData[_PAYLOAD_Acceleration_ + 5], rxData[_PAYLOAD_Acceleration_ + 6]) / 1000.f;
	}
}

int16_t IMU::Get_date(uint8_t LowBits, uint8_t HighBits)
{
	return (HighBits << 8) + LowBits;
}
float IMU::GetAngleYaw()
{
	return angle.yaw;
}

void  IMU::crc16_update(uint16_t* currectCrc, const uint8_t* src, uint32_t lengthInBytes)
{
	uint32_t crc = *currectCrc;
	uint32_t j;
	for (j = 0; j < lengthInBytes; ++j)
	{
		uint32_t i;
		uint32_t byte = src[j];
		crc ^= byte << 8;
		for (i = 0; i < 8; ++i)
		{
			uint32_t temp = crc << 1;
			if (crc & 0x8000)
			{
				temp ^= 0x1021;
			}
			crc = temp;
		}
	}
	*currectCrc = crc;
}
//extern Control ctrl;