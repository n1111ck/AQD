#ifndef CSTATES_H
#define CSTATES_H

namespace AQD
{
	namespace Simulation
	{
		enum EValidityState
		{
			EValidityState_Invalid = 1,
			EValidityState_Valid = 3
		};
		struct SGeneralStateData 
		{
			double Longitude;
			double Latitude;
			double Altitude;
			double Pitch;
			double Roll;
			double Yaw;
		};
		struct SSimulationParameters
		{
			double Mass;
			double InertiaX;
			double InertiaY;
			double InertiaZ;
			double Length;
			double WindForce;
			double WindDirection;
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
		};
		struct SFullStateData
		{
			struct SGeneralStateData PositionData;
			struct SGeneralStateData SpeedData;
			struct SGeneralStateData AccelData;
			struct SSimulationParameters SimParameters;
			struct SInputs Inputs;
		};
	}
}

#endif