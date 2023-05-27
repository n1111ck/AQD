#include "CDynamics.h"
#include <math.h>

const double c_Gravity = 9.81; // m/s2

double __Longitude(AQD::Simulation::SFullStateData fs)
{
	return 1 / fs.SimParameters.Mass *
		(
			(fs.Inputs.Force.Front + fs.Inputs.Force.Back) * sin(-fs.PositionData.Pitch) * cos(fs.PositionData.Yaw)
			+
			(fs.Inputs.Force.Left + fs.Inputs.Force.Right) * sin(fs.PositionData.Roll) * sin(fs.PositionData.Yaw)
			+
			fs.SimParameters.WindForce * cos(fs.SimParameters.WindDirection)
		);
}

double __Latitude(AQD::Simulation::SFullStateData fs)
{
	return 1 / fs.SimParameters.Mass *
		(
			(fs.Inputs.Force.Left + fs.Inputs.Force.Right) * sin(-fs.PositionData.Roll) * cos(fs.PositionData.Yaw)
			+
			(fs.Inputs.Force.Front + fs.Inputs.Force.Back) * sin(-fs.PositionData.Pitch) * sin(fs.PositionData.Yaw)
			+
			fs.SimParameters.WindForce * sin(fs.SimParameters.WindDirection)
		);
}

double __Altitude(AQD::Simulation::SFullStateData fs)
{
	return 1 / fs.SimParameters.Mass *
		(
			(fs.Inputs.Force.Front + fs.Inputs.Force.Back) * cos(fs.PositionData.Pitch)
			+
			(fs.Inputs.Force.Left + fs.Inputs.Force.Right) * cos(fs.PositionData.Roll)
			+
			fs.SimParameters.WindForce * sin(fs.SimParameters.WindDirection)
		) - c_Gravity;
}

double __Pitch(AQD::Simulation::SFullStateData fs)
{
	return fs.SimParameters.Length / fs.SimParameters.InertiaY * (fs.Inputs.Force.Front - fs.Inputs.Force.Back);
}

double __Roll(AQD::Simulation::SFullStateData fs)
{
	return fs.SimParameters.Length / fs.SimParameters.InertiaX * (fs.Inputs.Force.Left - fs.Inputs.Force.Right);
}

double __Yaw(AQD::Simulation::SFullStateData fs)
{
	return 1 / fs.SimParameters.InertiaX * (fs.Inputs.Torque.Front + fs.Inputs.Torque.Back - fs.Inputs.Force.Right - fs.Inputs.Force.Left);
}