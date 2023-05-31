#ifndef CPROPERTIES_H
#define CPROPERTIES_H

namespace AQD
{
	namespace Prop
	{
		enum EState
		{
			Longitude = 0,
			Latitude = 1,
			Altitude = 2,
			Pitch = 3,
			Roll = 4,
			Yaw = 5
		};
		struct SState
		{
			double Accel;
			double Speed;
			double Position;
		};
		struct SAttitude
		{
			struct SState Longitude;
			struct SState Latitude;
			struct SState Altitude;
			struct SState Pitch;
			struct SState Roll;
			struct SState Yaw;
		};
		struct SPropertyByAxis
		{
			double X;
			double Y;
			double Z;
		};
		struct SPropertyByPropeller
		{
			double Front;
			double Left;
			double Back;
			double Right;
		};
		struct SInputs
		{
			struct SPropertyByPropeller Force;
			struct SPropertyByPropeller Torque;
			struct SPropertyByPropeller Speed;
		};
	}
}

#endif