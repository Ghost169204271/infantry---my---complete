#include"control.h"
void Control::Init(std::vector<Motor*> chassis,std::vector<Motor*> pantile,Android* android,std::vector<Motor*> shooter)
{
	this->chassis.m_chassisMode = Control::Chassis::FOLLOW;
	for (size_t i = 0; i < chassis.size(); ++i)
	{
		this->chassis.chassis[i] = chassis[i];
		switch (chassis[i]->m_motor_use)
		{
		case chassisMotor:
			chassis[i]->setspeed = 0;
			chassis[i]->current = 0;
			break;
		default:
			break;
		}
	}
	for (size_t i = 0; i < pantile.size(); ++i)
	{
		this->pantile.pantile[i] = pantile[i];
		switch (pantile[i]->m_motor_use)
		{
		case pitchMotor:
			(pantile[i])->setangle = ((this->pantile).pitchmax + (this->pantile).pitchmin) / 2;
			break;
		case yawMotor:
			(pantile[i])->setangle = (this->pantile).setyaw;
			break;
		default:
			break;
		}
	}
	for (size_t i = 0; i < shooter.size(); ++i)
	{
		this->shooter.shooter[i] = shooter[i];
		switch (shooter[i]->m_motor_use)
		{
		case hitMotor:
			shooter[i]->setangle = this->shooter.setHitMotor;
			shooter[i]->spinning = false;
			shooter[i]->curcircle = 0;
			shooter[i]->stopAngle = this->shooter.setHitMotor;
			break;
		case rubMotor:
			shooter[i]->setangle == this->shooter.setLeftMotorSpeed;
			break;
		default:
			break;
		}

	}
	this->pantile.android = android;
}
//call me at 1khz

void Control::follow_speed(const int16_t speedx, const int16_t speedy)//使云台方向为行进的正方向
{
	float theta = 2 * PI * (pantile.pantile[0]->angle[now] - pantile.midyaw) / 8192.f;
	float st = sin(theta);
	float ct = cos(theta);
	chassis.speedx = speedx * ct - speedy * st;
	chassis.speedy = speedx * st + speedy * ct;

	if (this->chassis.m_chassisMode == Control::Chassis::LOCK)
	{
		float theat = pantile.midyaw - pantile.pantile[0]->angle[now];

		if (theat <= -4096.f)
		{
			theat += 8192.f;
		}
		else if (theat >= 4096.f)
		{
			theat -= 8192;
		}
		if (theat <= -111)
			chassis.speedz += 1000.f* getK(theat)*2.f;
		if (theat >= 111)
			chassis.speedz -= 1000.f * getK(theat)*2.f;
	}
}

void Control::manual_chassis(const int32_t speedx, const int32_t speedy, int32_t speedz)
{
	chassis.speedy = speedy;
	chassis.speedx = speedx;
	chassis.speedz = speedz;
}


void Control::manual_pantile(float32_t ch_yaw, float32_t ch_pitch)
{	

	float adjangle = this->pantile.sensitivity*2;
	if (this->chassis.m_chassisMode == Control::Chassis::FOLLOW)
	{		
		this->pantile.setpitch -= (float)(adjangle * ch_pitch);
		this->pantile.setyaw -= (float)(adjangle * ch_yaw );
	}
	else if(this->chassis.m_chassisMode == Control::Chassis::ROTATION)
	{
		imu_pantile.initialYaw = getDelta( imu_pantile.initialYaw -(adjangle * ch_yaw/22.75f));

		keepPantileYaw(imu_pantile.initialYaw, ch_yaw,imu_pantile);
		this->pantile.setpitch -= (float)(this->pantile.sensitivity * ch_pitch);
	}
	else if (this->chassis.m_chassisMode == Control::Chassis::LOCK)
	{
		keepPantileYaw(ctrl.pantile.midyaw, ch_yaw, pantile);
		this->pantile.setpitch -= (float)(this->pantile.sensitivity * ch_pitch);
	}
}
void Control::manual_shoot(bool shoot, bool auto_shoot,bool openRub)
{
	this->shooter.shoot = shoot;
	this->shooter.auto_shoot = auto_shoot;
	this->shooter.openRub = openRub;
}

void Control::Chassis::Update()
{
		ctrl.follow_speed(speedx, speedy);
		chassis[0]->setspeed = Ramp(+speedx + speedy - speedz, chassis[0]->setspeed);
		chassis[1]->setspeed = Ramp(-1 * (-speedx + speedy + speedz), chassis[1]->setspeed);
		chassis[2]->setspeed = Ramp(-1 * (+speedx + speedy + speedz), chassis[2]->setspeed);
		chassis[3]->setspeed = Ramp(-speedx + speedy - speedz, chassis[3]->setspeed);
	/*
	* the value of speedx,speedy and speedz are determined by function manual_chassis which is defined in control.h
	* the function of OnRC which is defined in RC.h updates the value
	*
	*/
}
void Control::Pantile::Update()
{
	if (setyaw > 8192.0)setyaw -= 8192.0;
	if (setyaw < 0.0)setyaw += 8192.0;
	
	setpitch = std::max(std::min(setpitch, pitchmax), pitchmin);
	int16_t anglepitch = setpitch, angleyaw = setyaw;
	if (angleyaw >= 8192)angleyaw -= 8192.0;
	if (angleyaw <= 0)angleyaw += 8192.0;

	if (anglepitch <= pitchmin)anglepitch = pitchmin;
	if (anglepitch >= pitchmax)anglepitch = pitchmax;

	//输出
	
	pantile[0]->setangle = angleyaw;
	pantile[1]->setangle = anglepitch;
}
void Control::Shooter::Update()
{
	//switch (judgement.data.ext_game_robot_status_t.shooter_id1_17mm_speed_limit)
	//{
	//case 15:
	//	speed = 4275;
	//	break;
	//case 18:
	//	speed = 4850;
	//	break;
	//case 30:
	//	speed = 7400;
	//	break;
	//default:break;
	//}
	if (openRub)
	{
		shooter[0]->setspeed = -speed;
		shooter[1]->setspeed = speed;
	}
	if (auto_shoot&&(!shoot))
	{
		shooter[0]->setspeed = -speed;
		shooter[1]->setspeed = speed;
		shooter[2]->spinning = true;
		shooter[2]->pd = false;
	}
	else if ((!auto_shoot)&&shoot)
	{
		shooter[0]->setspeed = -speed;
		shooter[1]->setspeed = speed;
		shooter[2]->spinning = false;
		shooter[2]->pd = true;
	}
	else if ((!shoot) && (!auto_shoot))
	{
		shooter[0]->setspeed = 0;
		shooter[1]->setspeed = 0;
		shooter[2]->setspeed = 0;
		shooter[2]->spinning = false;
		shooter[2]->pd = false;
	}

}
void Control::Update()
{
	chassis.Update();
	pantile.Update();
	shooter.Update();
	chassis.PowerUpdate();
}
void Control::keepPantileYaw(float angleKeep,float ch_yaw, IMU frameOfReference)
{
	float deltaYaw = 0, adjust = this->pantile.sensitivity;
	deltaYaw = getDelta(angleKeep - frameOfReference.GetAngleYaw())*22.75f;
	static float test;
	test = deltaYaw;
	if (deltaYaw && abs(ch_yaw) < 2.f)
	{
		if (deltaYaw < 0)
			ch_yaw = -1.f;
		if (deltaYaw > 0)
			ch_yaw = 1.f;
	}
	this->pantile.setyaw += ch_yaw * adjust * getK(deltaYaw);
}
void Control::keepPantileYaw(float angleKeep, float ch_yaw, Pantile frameOfReference)
{
	float deltaYaw = 0.f, adjust = this->pantile.sensitivity;
	deltaYaw = angleKeep  - (float)frameOfReference.pantile[0]->angle[now];
	if (deltaYaw <= -4096.f)
		deltaYaw += 8192.f;
	else if (deltaYaw >= 4096.f)
		deltaYaw -= 8291.f;

	if (deltaYaw && abs(ch_yaw) < 2.f)
	{
		if (deltaYaw < 0)
			ch_yaw = -1.f;
		if (deltaYaw > 0)
			ch_yaw = 1.f;
	}
		this->pantile.setyaw += ch_yaw * adjust/2.f*getK(deltaYaw);
}

int16_t Control::setrange(const int16_t original, const int16_t range)
{
	return fmaxf(fminf(range, original), -range);
}
/*
 * 	static int16_t setrange(const int16_t original, const int16_t range);
 *	this function is not called anywhere
 *	2022/01/17
 */

float Control::Chassis::getPowerLimit(int level)
{
	if (level == 0)return 40;
	else if (level == 1)return 60;
	else if (level == 2)return 80;
	else if (level == 3)return 100;
}
/*
* 	float getPowerLimit(int level);
*	this function is not called anywhere
*	2022/01/17
*/
void Control::Chassis::PowerUpdate()
{
	float fTotalBuffer = 60;
	float fCurrentBuffer = judgement.data.ext_power_heat_data_t.chassis_power_buffer;
	//judgement.data.ext_game_robot_status_t.robot_level
	if (judgement.powerheatready)
	{
		float deltaMaxCurrent = power.Delta(fCurrentBuffer - fTotalBuffer);
		chassis[0]->maxcurrent = std::max(13000 + deltaMaxCurrent, 0.f);
		chassis[1]->maxcurrent = std::max(13000 + deltaMaxCurrent, 0.f);
		chassis[2]->maxcurrent = std::max(13000 + deltaMaxCurrent, 0.f);
		chassis[3]->maxcurrent = std::max(13000 + deltaMaxCurrent, 0.f);
	}
}
float getDelta(float delta)
{
	if (delta <= -180.f)
	{
		delta += 360.f;
	}

	if (delta > 180.f)
	{
		delta -= 360.f;
	}
	return delta;
}
float Ramp(float setval, float curval,float RampSlope)
{
	if ((setval - curval) >= 0)
	{
		curval += RampSlope;
		curval = std::min(curval, setval);
	}
	else
	{
		curval -= RampSlope;
		curval = std::max(curval, setval);
	}
	return curval;
}
float getK(float de)
{
	if (abs(de) <= 22.75f)
		return 0.f;
	if(abs(de)<660)
	return abs(de) / 22.75f * 0.01f;
	return abs(de) / 22.75f * 0.02f;
}

//float get_k_delta(float delta)
//{
//	if (delta <= 1 && delta >= -1)
//		return 0;
//	if ((delta < -1 && delta >= -10) || (delta > 1 && delta <= 10))
//		return 0.1;
//	else if ((delta < -10 && delta >= -20) || (delta > 10 && delta <= 20))
//		return 0.15;
//	else if ((delta < -20 && delta >= -30) || (delta > 20 && delta <= 30))
//		return 0.2;
//	else if ((delta < -30 && delta >= -50) || (delta > 30 && delta <= 50))
//		return 0.4;
//	else if ((delta < -50 && delta >= -60) || (delta > 50 && delta <= 60))
//		return 0.5;
//	else
//		return 0.7f;
//}

