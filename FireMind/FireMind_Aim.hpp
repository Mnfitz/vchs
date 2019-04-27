
#include <Arduino.h>

class Aim
{
public:
    void Setup();
    void Loop();
};

inline void Aim::Setup()
{
    //code goes here
}

inline void Aim::Loop(Calc& inCalc)
{
	float horiz{};
	float vert{};
	const bool didGet = inCalc.GetBearing(horiz, vert);
	if (didGet)
	{
		printf("Horizontal Bearing: %f, Vertical Bearing: %f\n",horiz, vert);
	}
		
    
}