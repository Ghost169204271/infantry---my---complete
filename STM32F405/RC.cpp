#include"RC.h"
void RC::OnIRQHandler(size_t rxSize)
{
	if (rxSize != 18)return;
	if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)return;

	rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
	rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
	rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
	rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
	if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
	if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
	if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
	if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

	rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;//s2
	rc.s[1] = ((m_frame[5] >> 4) & 0x03);//s1

	pc.x = ((int16_t)m_frame[6]) | (((int16_t)m_frame[7]) << 8);
	pc.y = ((int16_t)m_frame[8]) | (((int16_t)m_frame[9]) << 8);
	pc.x= kalman[0].Filter(pc.x*1.f);
	pc.y = KalmanRe[1].Filter(kalman[1].Filter(pc.y * 1.f));

	pc.z = ((int16_t)m_frame[10]) | (((int16_t)m_frame[11] << 8));
	pc.press_l = m_frame[12];
	pc.press_r = m_frame[13];


	pc.key_h = m_frame[15];
	pc.key_l = m_frame[14];

	
}

void RC::OnRC()
{
	if (rc.s[0] == mid_position)
	{

		switch (rc.s[1])
		{
		case up_position:
			ctrl.manual_shoot(false, true,true);
			break;
		case mid_position:

			ctrl.manual_shoot(false, false, false);
			if (rc.ch[0] || rc.ch[1])
			{
				if (!lock_mode)
				{
					lock_mode = true;
					top_mode = false;
					follow_mode = false;
					ctrl.chassis.m_chassisMode = Control::Chassis::LOCK;
				}
				ctrl.manual_chassis(rc.ch[0] * MAXSPEED / 660, rc.ch[1] * MAXSPEED / 660, -rc.ch[2] * MAXSPEED / 1320);
				ctrl.manual_pantile(0, rc.ch[3] / 660.f);
			}
			else if (!rc.ch[0] && !rc.ch[1])
			{
				if (!follow_mode)
				{
					ctrl.chassis.m_chassisMode = Control::Chassis::FOLLOW;
					top_mode = false;
					follow_mode = true;
					lock_mode = false;
				}
				ctrl.manual_chassis(rc.ch[0] * MAXSPEED / 660, rc.ch[1] * MAXSPEED / 660,0);
				ctrl.manual_pantile(rc.ch[2] / 660.f, rc.ch[3] / 660.f);
			}

			break;
		case down_position:
			if (!top_mode)
			{
				imu_pantile.initialYaw = imu_pantile.GetAngleYaw();
				top_mode = true;//进入小陀螺模式
				follow_mode = false;
				lock_mode = false;
				ctrl.chassis.m_chassisMode = Control::Chassis::ROTATION;
			}
			ctrl.manual_shoot(false, false, false);

			ctrl.manual_chassis(rc.ch[0] * MAXSPEED / 660, rc.ch[1] * MAXSPEED / 660, 1500.f);
			ctrl.manual_pantile(rc.ch[2]  / 660.f, rc.ch[3] / 660.0);
			break;
		default:
			break;
		}
	}
	else if (rc.s[0] == down_position)
	{
		ctrl.manual_chassis(0, 0,0);
		ctrl.pantile.setpitch = (ctrl.pantile.pitchmax + ctrl.pantile.pitchmin) / 2;
		ctrl.pantile.setyaw = ctrl.pantile.midyaw;
		ctrl.manual_shoot(false, false, false);
	}
	
}
//pc.press_l：鼠标左键 pc.press_r：鼠标右键 pc.x:鼠标左右平移 pc.y:鼠标前后平移,朝己为正 pc.key_l，pc.key_h：按键
//底盘跟随模式：ctrl.follow_speed();
//				ctrl.pantile.follow = true;
//				ctrl.direction_speed();
//小陀螺模式：  ctrl.direction_speed();
//              ctrl.chassis.speedz = adjspeed;给speedz设定一个值

void RC::OnPC()
{
	if (rc.s[0] == up_position)
	{
		const float adjspeed = MAXSPEED;

		if (rc.s[1] == up_position)
		{
			ctrl.manual_shoot(pc.press_l, pc.press_r, true);
		}
		else if(rc.s[1]==mid_position)
		{
			ctrl.manual_shoot(pc.press_l, pc.press_r, (pc.press_l || pc.press_r));
		}
		else if (rc.s[1] == down_position)
		{
			ctrl.manual_shoot(pc.press_l, pc.press_r, (pc.press_l || pc.press_r));
			if (!follow_mode)
			{
				ctrl.chassis.m_chassisMode = Control::Chassis::FOLLOW;
				top_mode = false;
				follow_mode = true;
				lock_mode = false;
			}
		}

		ctrl.chassis.speedx = ctrl.chassis.speedy = ctrl.chassis.speedz = 0;
		if (pc.key_l & Q)
		{
			if (!top_mode)
			{
				imu_pantile.initialYaw = imu_pantile.GetAngleYaw();
				top_mode = true;//进入小陀螺模式
				follow_mode = false;
				lock_mode = false;
				ctrl.chassis.m_chassisMode = Control::Chassis::ROTATION;
			}
		}
		if (pc.key_l & E)
		{	
			ctrl.chassis.speedz -= adjspeed / 5;
			ctrl.chassis.m_chassisMode = Control::Chassis::FOLLOW;
			top_mode = false;//退出小陀螺模式
			follow_mode = true;
			lock_mode = false;
		}

		if (pc.key_h & Z)
		{
			pc.prePressZ = true;

		}
		if (pc.prePressZ && !(pc.key_h & Z))
		{
				if (!lock_mode)
				{
					lock_mode = true;
					top_mode = false;
					follow_mode = false;
					ctrl.chassis.m_chassisMode = Control::Chassis::LOCK;
				}
				else if (lock_mode)
				{
					lock_mode = false;
					follow_mode = true;
					top_mode = false;
					ctrl.chassis.m_chassisMode = Control::Chassis::FOLLOW;
				}
				pc.prePressZ = false;
		}

		if (top_mode||follow_mode|| lock_mode)
		{
			pc.direct = ((!!(pc.key_l & W)) + (!!(pc.key_l & S)) * (-1));
			pc.setY = pc.direct * (adjspeed / 6);
			pc.direct = ((!!(pc.key_l & D)) + (!!(pc.key_l & A)) * (-1));
			pc.setX = pc.direct * (adjspeed / 6);
			ctrl.manual_chassis(pc.setX, pc.setY);
		}

		if (pc.key_l & SHIFT&&top_mode)ctrl.chassis.speedz += adjspeed/5;//底盘旋转

		if (fix)
		{

		}
		else
		{
			if (follow_mode || top_mode)
				ctrl.manual_pantile(+pcGetMove(pc.x), -pcGetMove(pc.y));
			else if (lock_mode)
			{
				ctrl.manual_chassis(ctrl.chassis.speedx, ctrl.chassis.speedy, ( - pc.x*25));
				ctrl.manual_pantile(0,-pcGetMove(pc.y));
			}
		}
	}
	else if(rc.s[0]==down_position)
	{
		ctrl.manual_chassis(0,0,0);
		ctrl.pantile.setpitch = (ctrl.pantile.pitchmax + ctrl.pantile.pitchmin) / 2;
		ctrl.pantile.setyaw = ctrl.pantile.midyaw;
		ctrl.manual_shoot(false, false, false);
	}
}

float32_t RC::pcGetMove(int32_t change)
{
	float32_t re = 1.0;
	if (change == 0)
		return 0;
	if (change < 0)
	{
		re *= -1.0;
	}
	if (change <= 10 && change >= -10)
	{
		re *= 0.1;
	}else if ((change > 10 && change <= 50) || (change >= -50 && change < -10))
	{
		re *= 0.2;
	}
	else if ((change>50&&change<=150)||(change>=-150&&change<-50))
	{
		re *= 0.5;
	}
	else if (change < -150 || change>150)
	{
		re *= 1.0;
	}
	return re;
}


void RC::Update()
{
	OnRC();
	OnPC();
}