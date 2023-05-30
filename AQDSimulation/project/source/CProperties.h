#ifndef CPROPERTIES_H
#define CPROPERTIES_H

namespace AQD
{
	namespace Prop
	{
		struct SMotor
		{
			double Kv;	// Constant that relates voltage with motor speed (linear)
			double Kf;	// Constant that relates motor speed with thrust (quadratic)
			double Kt;	// Constant that relates motor speed with torque (quadratic)
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
			double x;
			double y;
			double z;
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