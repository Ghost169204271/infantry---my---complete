#pragma once

#include <stm32f4xx_hal.h>
#include"PID.h"
#include"kalman.h"

#define _PACKET_PRE_ (0)
#define _PACKET_TYPE_ (_PACKET_PRE_+1)
#define _PACKET_LEN_ (_PACKET_TYPE_+1)
#define _PACKET_CRC_ (_PACKET_LEN_+2)
#define _PACKET_PAYLOAD_ (_PACKET_CRC_+2)
#define _PAYLOAD_USER_ID_ (_PACKET_PAYLOAD_)
#define _PAYLOAD_Acceleration_ (_PAYLOAD_USER_ID_+2)
#define _PAYLOAD_AngularVelocity_ (_PAYLOAD_Acceleration_+7)
#define _PAYLOAD_EulerAngles_ (_PAYLOAD_AngularVelocity_+14)

constexpr uint32_t MAX_DateLength = 41;

typedef enum
{
	PRE = 0x5A,
	TYPE = 0xA5,
	Pakage_ID = 0x90,
	User_ID = 0x0,
	Acceleration_ID = 0xA0,
	AngularVelocity_ID = 0xB0,
	EulerAngles_ID = 0xD0
}ItemID_t;
enum 
{
	YAW=0,PITCH,ROLL
};
typedef struct
{
		float pitch,yaw,roll;
}Angle, AngularVelocity, Acceleration;


class IMU
{
public:
	void Init();
	uint8_t* GetDMARx();
	void OnIRQHandler(size_t rxSize);
	float initialYaw;
	float GetAngleYaw();
private:	
	void crc16_update(uint16_t* currectCrc, const uint8_t* src, uint32_t lengthInBytes);
	int16_t Get_date(uint8_t LowBits,uint8_t HighBits);
	bool crc_test(uint8_t* date_IMU);	
	AngularVelocity angularVelocity;
	Kalman imuKalman[3] = { {1.f,40.f},{1.f,40.f},{1.f,40.f} };
	Acceleration acceleration;
	Angle angle;
	uint8_t rxData[MAX_DateLength];
	bool state = 0;
};

extern IMU imu_pantile, imu_chassis;