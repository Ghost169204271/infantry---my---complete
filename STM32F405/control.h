#ifndef __CONTROL__
#define __CONTROL__
#include <stm32f4xx_hal.h>
#include <vector>
#include "IMU.h"
#include "motor.h"
#include "tim.h"
#include <cmath>
#include "NUC.h"
#include "judgement.h"
#include "Android.h"

float Ramp(float setval, float curval, float RampSlope=20.f);
float getDelta(float delta);
float getK(float de);
class Control final
{
public:
	struct Chassis
	{
		enum ChassisMode
		{
			LOCK,FOLLOW,ROTATION
		};
		Motor *chassis[4];
		int16_t speedx=0, speedy=0, speedz=0;
		PID power{ 30.0f, 200.f, 2000.f};

		float fRampSlope = 20;
		ChassisMode m_chassisMode = LOCK;
		float bias = sqrt(2);
		void Update();
		float getPowerLimit(int level);
		/*
		* 	float getPowerLimit(int level);
		*	this function is not called anywhere
		*	2022/01/17
		*/
		void PowerUpdate();
	};
	struct Pantile
	{
		Motor   *pantile[2];
		Android *android;
		const uint16_t midyaw = 5500;//yaw轴初始角度
		float pitchmax = 4800, pitchmin = 3350;//pitch轴限位
		float setpitch = (pitchmax+pitchmin)/2, setyaw = midyaw;//pitch轴初始角度
		const float sensitivity = 5.f;
		bool follow = true;
		bool pre_follow = false;
		bool aim = false;
		enum { YAW, PITCH };

		void Update();
	};
	struct Shooter
	{
		Motor *shooter[3];
		int16_t speed = 4500;//speed: 最速时电流大小
		int16_t setLeftMotorSpeed =0 , setRightMotorSpeed=0;
		float setHitMotor = 3910;
		bool openRub = false;
		bool shoot = false;
		bool auto_shoot = false;

		//the following three parameters are used for for automatic aiming
		bool fraction = false;
		bool fullheat_shoot = false;
		bool heat_ulimit = false;

		void Update();
	private:
		uint32_t counter = 0;
	};

	Chassis chassis{};
	Pantile pantile{};
	Shooter shooter{};
	void follow_speed(const int16_t speedx, const int16_t speedy);

	void manual_chassis(const int32_t speedx, const int32_t speedy,  int32_t speedz=0);

	void keepPantileYaw(float angleKeep, float ch_yaw,IMU frameOfReference);
	void keepPantileYaw(float angleKeep, float ch_yaw, Pantile frameOfReference);

	void manual_shoot(bool shoot, bool auto_shoot,bool openRub);
	void manual_pantile(float32_t ch_yaw, float32_t ch_pitch);//ch_yaw*sensitivity为改变量
	void Init(std::vector<Motor*> chassis,std::vector<Motor*> pantile,Android* android,std::vector<Motor*> shooter);
	//call me at 1khz
	void Update();
	static int16_t setrange(const int16_t original, const int16_t range);
	/*
	 * 	static int16_t setrange(const int16_t original, const int16_t range);
	 *	this function is not called anywhere
	 *	2022/01/17
	 */
};

extern Control ctrl;
extern Judgement judgement;

#endif