#ifndef CSTATES_H
#define CSTATES_H

namespace AQD
{
	namespace Simulation
	{
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
		struct SSimulationParameters
		{
			double Mass;
			double Inertia[3];
			double PolarInertia;
			double Length;
			double WindForce;
			double WindDirection;
			double Step;
			double Time;
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
		struct SFullStateData
		{
			struct SAttitude Attitude;
			struct SSimulationParameters SimParameters;
			struct SInputs Inputs;
		};
	}
}

#endif