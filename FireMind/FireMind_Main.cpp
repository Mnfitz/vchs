// Arduino
#include <Arduino.h>
#include <assert.h>

// FireMind
#include "FireMind_GpsCoords.hpp"
#include "FireMind_Recv.hpp"
#include "FireMind_Calc.hpp"
#include "FireMind_Aim.hpp"

int main()
{
	init(); //initialize timers

#if defined(USBCON)
	USBDevice.attach();
#endif

#if 0
	Serial.begin(115200);
	Serial.println("Hello, world");
	Serial.flush(); //let serial prinitng finish
#else
	Recv recv;
	Calc calc;
	Aim  aim;

	recv.Setup();
	calc.Setup();
	aim.Setup();

	for (;;)
	{
		recv.Loop();
		calc.Loop(recv);
		aim.Loop(calc);
		if (serialEventRun)
		{
			serialEventRun();
		}
	}
#endif

} // main()