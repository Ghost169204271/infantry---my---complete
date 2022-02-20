#pragma once
#include "usart.h."
#include "control.h"

#define mid_position  (3)
#define up_position  (1)
#define down_position  (2)

#define W     ((uint8_t)0x01<<0)
#define S     ((uint8_t)0x01<<1)
#define A     ((uint8_t)0x01<<2)
#define D     ((uint8_t)0x01<<3)
#define SHIFT ((uint8_t)0x01<<4)
#define CTRL  ((uint8_t)0x01<<5)
#define Q     ((uint8_t)0x01<<6)
#define E     ((uint8_t)0x01<<7)

#define R     ((uint8_t)0x01<<0)
#define F     ((uint8_t)0x01<<1)
#define G     ((uint8_t)0x01<<2)
#define Z     ((uint8_t)0x01<<3)
#define X     ((uint8_t)0x01<<4)
#define C     ((uint8_t)0x01<<5)

class RC
{
public:
	bool follow_mode = true;
	bool top_mode = false;
	bool lock_mode = false;
	bool fix = false;
	struct
	{
		int16_t ch[4];
		uint8_t s[2];
	}rc;
	struct
	{
		int16_t x, y, z;
		uint8_t press_l, press_r, key_h, key_l;
		bool topmodeJudge = false;
		int8_t direct;
		int32_t setX, setY;
		bool prePressZ=0;
		const float spdratio = 10.f;
	}pc;
	uint8_t* GetDMARx(void) { return m_frame; }

	void OnIRQHandler(size_t rxSize);

	void OnRC();
	//pc.press_l�������� pc.press_r������Ҽ� pc.x:�������ƽ�� pc.y:���ǰ��ƽ�� pc.key_l��pc.key_h������
	//���̸���ģʽ��ctrl.follow_speed();
	//				ctrl.pantile.follow = true;
	//				ctrl.direction_speed();
	//С����ģʽ��  ctrl.direction_speed();
	//              ctrl.chassis.speedz = adjspeed;��speedz�趨һ��ֵ
	void OnPC();
	Kalman kalman[2] = { { 1.f, 20.f }, { 1.f, 20.f } }, KalmanRe[2] = { {1.f,30.f}, {1.f,30.f} };
//	float setyaw = 0, setpitch = 0;
	void Update();
	float32_t pcGetMove(int32_t change);
private:
	uint8_t m_frame[UART_MAX_LEN]{};
};

extern RC rc;
