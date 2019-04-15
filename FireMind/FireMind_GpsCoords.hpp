
class GpsCoords
{
public:
    float mLongitude{};
    float mLatitude{};
    float mAltitude{};

    GpsCoords(); //Default Ctor
    GpsCoords(float inLon, float inLat, float inAlt); //Non-Default Ctor
};

inline GpsCoords::GpsCoords() :
    mLongitude{0},
    mLatitude{0},
    mAltitude{0}
{
    //Nothing else to do
}

inline GpsCoords::GpsCoords(float inLon, float inLat, float inAlt) :
    mLongitude{inLon},
    mLatitude{inLat},
    mAltitude{inAlt}
{
    //Nothing else to do
}