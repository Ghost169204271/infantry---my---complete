#pragma once
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <cmath>
#include "PID.h"
#include "kalman.h"

constexpr auto MAXSPEED = 5000;
//constexpr auto ADJUSTSPEED = 3000;

enum { ID1 = 0x201, ID2, ID3, ID4, ID5, ID6, ID7, ID8};
enum { pre = 0, now };
enum pid_mode{ speed = 0, position };
enum motor_type{ M3508, M3510, M2310, EC60, M6623, M6020,M2006 };
enum motor_mode { SPD, POS, ACE };
enum motor_use{pitchMotor,yawMotor,hitMotor,rubMotor,chassisMotor};
#define SQRTF(x) ((x)>0?sqrtf(x):-sqrtf(-x))			
#define T 1.e-3f

class Motor
{
	typedef motor_type type_t;
public:
	uint32_t ID;
	Kalman kalman{ 1.f,40.f };
	Motor(const type_t type, const motor_mode mode,const motor_use use ,const uint32_t id, PID _speed, PID _position);
	Motor(const type_t type, const motor_mode mode,  const motor_use use , const uint32_t id, PID _speed);
	void Ontimer(uint8_t idata[][8], uint8_t* odata);
private:
	void getmax(const type_t type);


	static int16_t getword(const uint8_t high, const uint8_t low);
	
	static int32_t setrange(const int32_t original, const int32_t range);
	type_t type;
public:
	static int16_t getdeltaa(int16_t diff);
	motor_use m_motor_use;
	int32_t current{}, curspeed{}, setspeed{},torque_current;//这个current用于输出电流或者电压
	int16_t adjspeed{};
	int16_t maxspeed{}, maxcurrent{};
	Kalman currentKalman{ 1.f,40.f };
	double setangle{}, angle[2]{};
	int32_t curcircle,stopAngle;
	int32_t mode{};
	bool pd = 0,spinning=0;//pd:单次拨弹 spinning:一秒八发
	PID pid[2];
};

//此处要根据实际不同can线上的电机数量进行更改
extern Motor can1_motor[5];
extern Motor can2_motor[4];
