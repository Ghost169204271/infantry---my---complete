#pragma once
#include <stm32f4xx_hal.h>
#include <arm_math.h>

class Android
{
public:
	typedef struct
	{
		float yaw, roll, pitch;
	}Angle;

	typedef struct
	{
		float aimx, aimy;
	}Aim;

	typedef struct
	{
		Angle angle;
		Aim aim;
	}InfoPack;

	InfoPack infopack;
	Aim aim{};
	Angle angle{};
	Angle offsetangle{};
	Angle raw_angle{}, pre_angle{};
	uint8_t rxData[99];

	void Init()
	{

	}
	static float wrapangle(const float original)noexcept
	{
		if (original < 0)return original + 360;
		else if (original > 360)return original - 360;
		else return original;
	}
	void OnIRQHandler(size_t rxSize)
	{
		memcpy(&infopack, rxData, sizeof(InfoPack));
		raw_angle = infopack.angle;
		aim = infopack.aim;

		angle.pitch = wrapangle(raw_angle.pitch - offsetangle.pitch);
		angle.yaw = wrapangle(raw_angle.yaw - offsetangle.yaw);
	}
	void SetOffset(float pitch, float yaw)
	{
		offsetangle.pitch = raw_angle.pitch - pitch;
		offsetangle.yaw = raw_angle.yaw - yaw;
	}
	uint8_t* GetDMARx()
	{
		return rxData;
	}
};

extern Android android;