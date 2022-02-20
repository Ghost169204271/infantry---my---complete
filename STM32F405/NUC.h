#pragma once

#include "usart.h"
#include "string"
#include "judgement.h"
#include "usb.h"

class NUC
{
public:
	typedef struct
	{
		float x, y, a;
	}datapack;

	float daimx, daimy, aimratio;
	datapack data;
	uint8_t rxData[100] = {};

	UART* uart = nullptr;
	USB* usb = nullptr;

	void Init(UART* uart)
	{
		this->uart = uart;
	}
	void Init(USB* usb)
	{
		this->usb = usb;
	}
	bool InformNUC(Judgement* judgement)
	{
		uint8_t type[] = "X";
		if (judgement->data.ext_game_robot_status_t.robot_id < 10)//self if RED
			type[0] = 'B';
		else //self is BLUE
			type[0] = 'R';
		if (this->usb)
		{
			usb->Transmit(type, 1);
		}
		if (this->uart)
		{
			uart->UARTTransmit(type, 1);
		}
	}

	uint8_t* GetDMARx(void)
	{
		return rxData;
	}

	void OnIRQHandler(size_t rxSize)
	{
		memcpy(&data, rxData, sizeof(datapack));
		daimx = data.x;
		daimy = data.y;
		aimratio = data.a;
	}
	void OnUSBRecive()
	{
		memcpy(&data, rxData, sizeof(datapack));
		daimx = data.x;
		daimy = data.y;
		aimratio = data.a;
	}
};

extern NUC nuc;	

