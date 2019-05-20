
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


class GpsXYZ
{
public:
	float mX{};
	float mY{};
	float mZ{};

	GpsXYZ(); //Default Ctor
	GpsXYZ(float inX, float inY, float inZ); //Non-Default Ctor
};

inline GpsXYZ::GpsXYZ() :
	mX{0},
	mY{0},
	mZ{0}
{
	//Nothing else to do
}

inline GpsXYZ::GpsXYZ(float inX, float inY, float inZ) :
	mX{inX},
	mY{inY},
	mZ{inZ}
{
	//Nothing else to do
}