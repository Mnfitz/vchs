
#include <Arduino.h>

class Calc
{
public:
	void Setup();
	void Loop(Recv& inRecv);
	bool GetBaseCoords(GpsCoords& outGpsCoords);

	float GetBearingHorizontal(const GpsCoords& inDegrees)

private:
	GpsCoords DegToRadCoords(const GpsCoords& inDegrees);
	GpsCoords RadToDegCoords(const GpsCoords& inRadians);
	GpsCoords MultCoords(const GpsCoords& inLhs, const GpsCoords& inRhs);
	GpsCoords SubCoords(const GpsCoords& inLhs, const GpsCoords& inRhs);


private:
	GpsCoords mBaseCoords{};
	float mCurrentBearing;

};

inline void Calc::Setup()
{ 
	//Will get the location of the base to me
	const bool getBase = GetBaseCoords(mBaseCoords);

	//The Motieno better give us a GPS coord
	//for the base, or else we can't calculate
	assert(getBase); 
}

inline void Calc::Loop(Recv& inRecv)
{

	GpsCoords targetCoords{};
	const bool getTarget = inRecv.GetCoords(targetCoords);
	if (getTarget)
	{
		//x1 y1 is the same as |mBaseCoords|
		//x2 y2 is the same as |targetCoords|
		/*

		float pi=3.14159265358979323846;
		float d;
		float r;
		
		float alt;
		float az;
		float el;
		
		float x1; 
		float y1;

		float x2; 
		float y2;

		d=r*acos(sin(y1)*sin(y2)+cos(y1)*cos(y2)*cos(x2-x1));

		az= ((90-atan2(sin(x2-x1),cos(y1)*tan(y2)-sin(y1)*cos(x2-x1)))*(180/pi));

		el=(atan2(alt,d)*(180/pi));
		*/
	}

}

inline bool Calc::GetBaseCoords(GpsCoords& outGpsCoords)
{
	//get coords from base moteino
	return true;
}

inline GpsCoords Calc::MultCoords(const GpsCoords& inLhs, const GpsCoords& inRhs) 
{
	GpsCoords multCoords{};
	multCoords.mLatitude = inLhs.mLatitude * inRhs.mLatitude; 
	multCoords.mLongitude = inLhs.mLongitude * inRhs.mLongitude;
	multCoords.mAltitude = inLhs.mAltitude * inRhs.mAltitude;

	return multCoords;
}

inline GpsCoords Calc::SubCoords(const GpsCoords& inLhs, const GpsCoords& inRhs) 
{
	GpsCoords subCoords{};
	subCoords.mLatitude = inLhs.mLatitude - inRhs.mLatitude; 
	subCoords.mLongitude = inLhs.mLongitude - inRhs.mLongitude;
	subCoords.mAltitude = inLhs.mAltitude - inRhs.mAltitude;

	return subCoords;
}

inline GpsCoords Calc::DegToRadCoords(const GpsCoords& inDegrees) 
{
	const float kPi = PI;
	const float kDegToRad = DEG_TO_RAD;
	const GpsCoords kToRadians{kDegToRad, kDegToRad, 1.0f};
	return MultCoords(inDegrees, kToRadians);
}

inline GpsCoords Calc::RadToDegCoords(const GpsCoords& inRadians) 
{
	const float kPi = PI;
	const float kRadToDeg = RAD_TO_DEG;
	const GpsCoords kToDegrees{kRadToDeg, kRadToDeg, 1.0f};
	return MultCoords(inRadians, kToDegrees);
}

inline float GetBearingHorizontal(const GpsCoords& inDegrees)
{
	//!!!!!!!!!!!!!!!!!!!
	//EMPTY FUNCTION BODY
	//!!!!!!!!!!!!!!!!!!!
}