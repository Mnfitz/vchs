
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
  GpsCoords deltaCoords = SubCoords(baseRadCoords, targetRadCoords);

  //Find a better naming convention pls
  float tempTargetLat = tan(inDegrees.mLatitude / 2 + PI / 4);
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
  return (targetBearing - mCurrentBearing);
}

inline float Calc::GetBearingVertical(const GpsCoords& inDegrees)
{
  GpsCoords baseRadCoords = DegToRadCoords(mBaseCoords);
  GpsCoords targetRadCoords = DegToRadCoords(inDegrees);
  GpsCoords deltaCoords = SubCoords(baseRadCoords, targetRadCoords);

  GpsCoords fromXDelta = MultCoords(baseRadCoords, deltaCoords);
  GpsCoords fromXFrom = MultCoords(baseRadCoords, deltaCoords);
  GpsCoords deltaXDelta = MultCoords(deltaCoords, deltaCoords);

  //Find a better naming convention pls
  float d =(fromXDelta.mLatitude + fromXDelta.mLongitude + fromXDelta.mAltitude);
  float a =(fromXFrom.mLatitude + fromXFrom.mLongitude + fromXFrom.mAltitude);
  float b =(deltaXDelta.mLatitude + deltaXDelta.mLongitude + deltaXDelta.mAltitude);

  float divisor = sqrt(a * b);
  if (abs(divisor) <= 0.0000001)
  {
    divisor = 0.0000001;
  }
  
  float elevation = RAD_TO_DEG * (acos(d / divisor));
  
  elevation = elevation - 90;

  return (elevation - mCurrentElevation);
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
