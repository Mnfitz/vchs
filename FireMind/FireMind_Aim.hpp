
#include <Arduino.h>

class Aim
{
public:
    void Setup();
    void Loop(Calc& inCalc);
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
    //Serial.print("Horizontal Bearing: ");
    //Serial.print(horiz);
    //Serial.print(", Vertical Bearing: ");
    //Serial.println(vert);
  }
    
    
}
