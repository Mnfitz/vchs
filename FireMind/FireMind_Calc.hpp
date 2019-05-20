
#include <Arduino.h>

class Calc
{
public:
  void Setup();
  void Loop(Recv& inRecv);
  bool GetBaseCoords(GpsCoords& outGpsCoords);
  bool GetBearing(float& outHoriz, float& outVert);

  float GetBearingHorizontal(const GpsCoords& inDegrees);
  float GetBearingVertical(const GpsCoords& inDegrees);

private:
  GpsCoords DegToRadCoords(const GpsCoords& inDegrees);
  GpsCoords RadToDegCoords(const GpsCoords& inRadians);
  GpsCoords MultCoords(const GpsCoords& inLhs, const GpsCoords& inRhs);
  GpsCoords SubCoords(const GpsCoords& inLhs, const GpsCoords& inRhs);
  
  GpsXYZ MultCoords(const GpsXYZ& inLhs, const GpsXYZ& inRhs);
  GpsXYZ SubCoords(const GpsXYZ& inLhs, const GpsXYZ& inRhs);
  float GetN(float inA, float inE, float inLatitude);
  GpsXYZ GetXYZ(const GpsCoords& inGpsCoord);

 private:
  GpsCoords mBaseCoords{};
  float mCurrentBearing{};
  float mCurrentElevation{};
  bool mIsHorizReady{false};
  bool mIsVertReady{false};
};

inline void Calc::Setup()
{ 
  //Serial.println("Calc Setup");
  //Will get the location of the base to me
  const bool getBase = GetBaseCoords(mBaseCoords);

  //The Motieno better give us a GPS coord
  //for the base, or else we can't calculate
  assert(getBase); 
}

inline void Calc::Loop(Recv& inRecv)
{
  //Serial.println("Calc Loop");
  GpsCoords targetCoords{};
  const bool didGetTarget = inRecv.GetCoords(targetCoords);
  if (didGetTarget)
  {
    const bool isBaseValid = (mBaseCoords.mLatitude != 0
                                && mBaseCoords.mLongitude != 0
                                && mBaseCoords.mAltitude != 0);

    if (!isBaseValid)
    {
      mBaseCoords = targetCoords;
    }
    mCurrentBearing = GetBearingHorizontal(targetCoords);
    mCurrentElevation = GetBearingVertical(targetCoords);
    mIsHorizReady = true;
    mIsVertReady = true;
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

inline GpsXYZ Calc::MultCoords(const GpsXYZ& inLhs, const GpsXYZ& inRhs) 
{
  GpsXYZ multCoords{};
  multCoords.mX = inLhs.mX * inRhs.mX; 
  multCoords.mY = inLhs.mY * inRhs.mY;
  multCoords.mZ = inLhs.mZ * inRhs.mZ;

  return multCoords;
}

inline GpsXYZ Calc::SubCoords(const GpsXYZ& inLhs, const GpsXYZ& inRhs)
{
  GpsXYZ subCoords{};
  subCoords.mX = inLhs.mX - inRhs.mX; 
  subCoords.mY = inLhs.mY - inRhs.mY;
  subCoords.mZ = inLhs.mZ - inRhs.mZ;

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

inline float Calc::GetBearingHorizontal(const GpsCoords& inDegrees)
{
  GpsCoords baseRadCoords = DegToRadCoords(mBaseCoords);
  GpsCoords targetRadCoords = DegToRadCoords(inDegrees);
  GpsCoords deltaCoords = SubCoords(targetRadCoords, baseRadCoords);

  //Find a better naming convention pls
  float tempTargetLat = tan(targetRadCoords.mLatitude / 2 + PI / 4);
  float tempBaseLat = tan(baseRadCoords.mLatitude / 2 + PI / 4);
  float deltaPhi = log(tempTargetLat / tempBaseLat);

  if(abs(deltaCoords.mLongitude) > PI)
  {
    if((deltaCoords.mLongitude) > 0.0)
    { 
      deltaCoords.mLongitude = -(2 * PI - deltaCoords.mLongitude);
    }
    else
    {
      deltaCoords.mLongitude = (2 * PI - deltaCoords.mLongitude);
    }
  }

  float targetBearing = fmod((RAD_TO_DEG * (atan2(deltaCoords.mLongitude, deltaPhi)) + 360.0), 360.0);
  return (targetBearing /*- mCurrentBearing*/);
}

inline float Calc::GetBearingVertical(const GpsCoords& inDegrees)
{
  GpsCoords baseRadCoords = DegToRadCoords(mBaseCoords);
  GpsCoords targetRadCoords = DegToRadCoords(inDegrees);
  GpsCoords deltaCoords = SubCoords(targetRadCoords, baseRadCoords);

  //GpsCoords fromXDelta = MultCoords(baseRadCoords, deltaCoords);
  //GpsCoords fromXFrom = MultCoords(baseRadCoords, deltaCoords);
  //GpsCoords deltaXDelta = MultCoords(deltaCoords, deltaCoords);

  GpsXYZ baseXYZ = GetXYZ(baseRadCoords);
  GpsXYZ targetXYZ = GetXYZ(targetRadCoords);
  
  GpsXYZ deltaXYZ = SubCoords(targetXYZ, baseXYZ);
  GpsXYZ baseDeltaXYZ = MultCoords(baseXYZ, deltaXYZ);
  
  baseXYZ = MultCoords(baseXYZ, baseXYZ);
  deltaXYZ = MultCoords(deltaXYZ, deltaXYZ);
  
  //Find a better naming convention pls
  float d =(baseDeltaXYZ.mX + baseDeltaXYZ.mY + baseDeltaXYZ.mZ);
  float a =(baseXYZ.mX + baseXYZ.mY + baseXYZ.mZ);
  float b =(deltaXYZ.mX + deltaXYZ.mY + deltaXYZ.mZ);

  float divisor = sqrt(a * b);
  if (abs(divisor) <= 0.0000001)
  {
    divisor = 0.0000001;
  }
  
  float elevation = RAD_TO_DEG * (acos(d / divisor));
  
  elevation = elevation - 90;

  return (/*mCurrentElevation*/ -elevation);
}

inline bool Calc::GetBearing(float& outHoriz, float& outVert)
{
  if (mIsHorizReady && mIsVertReady)
  {
    outHoriz = mCurrentBearing;
    outVert = mCurrentElevation;
    
    mIsHorizReady = false;
    mIsVertReady = false;
    return (true);
  }

  return (false); 
}

inline float Calc::GetN(float inA, float inE, float inLatitude)
{
	float sinLatitude = sin(inLatitude);
	float denom = sqrt(1 - inE * inE * sinLatitude * sinLatitude);
	return inA/denom;	
}

inline GpsXYZ Calc::GetXYZ(const GpsCoords& inGpsCoord)
{
	float radius = 6378137;
	float flatteningDenom = 298.257223563;
	float flattening = 0.003352811;
	float polarRadius = 6356752.312106893;

	float asqr = (radius * radius);
	float bsqr = polarRadius * polarRadius;
	float e = sqrt((asqr-bsqr)/asqr);

	float N = GetN(radius, e, inGpsCoord.mLatitude);
	float ratio = bsqr / asqr;

	float X = (N + inGpsCoord.mAltitude) * cos(inGpsCoord.mLatitude) * cos(inGpsCoord.mLongitude);
	float Y = (N + inGpsCoord.mAltitude) * cos(inGpsCoord.mLatitude) * sin(inGpsCoord.mLongitude);
	float Z = (ratio * N + inGpsCoord.mAltitude) * sin(inGpsCoord.mLatitude);

	GpsXYZ returnXYZ{X,Y,Z};


	return returnXYZ;
}
