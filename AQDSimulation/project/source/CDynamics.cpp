#include "CDynamics.h"
#include <cstdlib>
#include <iostream>

const double c_Gravity = 9.81; // m/s2

double AQD::Simulation::__Longitude(SState state, AQD::Simulation::SFullStateData fs)
{
	return (fs.Inputs.Force.Front + fs.Inputs.Force.Left + fs.Inputs.Force.Back + fs.Inputs.Force.Right) / fs.SimParameters.Mass *
		(
			std::sin(fs.Attitude.Yaw.Position) * std::sin(fs.Attitude.Roll.Position)
			+ 
			std::cos(fs.Attitude.Yaw.Position) * std::cos(fs.Attitude.Roll.Position) * std::sin(fs.Attitude.Pitch.Position)
		);
}

double AQD::Simulation::__Latitude(SState state, AQD::Simulation::SFullStateData fs)
{
	return (fs.Inputs.Force.Front + fs.Inputs.Force.Left + fs.Inputs.Force.Back + fs.Inputs.Force.Right) / fs.SimParameters.Mass *
		(
			std::sin(fs.Attitude.Yaw.Position) * std::cos(fs.Attitude.Roll.Position) * std::sin(fs.Attitude.Pitch.Position)
			-
			std::cos(fs.Attitude.Yaw.Position) * std::sin(fs.Attitude.Roll.Position)
		);
}

double AQD::Simulation::__Altitude(SState state, AQD::Simulation::SFullStateData fs)
{
	return (fs.Inputs.Force.Front + fs.Inputs.Force.Left + fs.Inputs.Force.Back + fs.Inputs.Force.Right) / fs.SimParameters.Mass * std::cos(fs.Attitude.Pitch.Position) * std::cos(fs.Attitude.Roll.Position)
		- 
		c_Gravity;
}

double AQD::Simulation::__Pitch(SState state, AQD::Simulation::SFullStateData fs)
{
	return 1 / fs.SimParameters.Inertia[0] * 
		(
			(fs.SimParameters.Inertia[1] - fs.SimParameters.Inertia[2])*fs.Attitude.Roll.Speed*fs.Attitude.Yaw.Speed
			-
			fs.SimParameters.PolarInertia * fs.Attitude.Roll.Speed * (fs.Inputs.Speed.Front - fs.Inputs.Speed.Left + fs.Inputs.Speed.Back - fs.Inputs.Speed.Right)
			+
			fs.SimParameters.Length * (fs.Inputs.Force.Back - fs.Inputs.Force.Front)
		);
}

double AQD::Simulation::__Roll(SState state, AQD::Simulation::SFullStateData fs)
{
	return 1 / fs.SimParameters.Inertia[1] *
		(
			(fs.SimParameters.Inertia[2] - fs.SimParameters.Inertia[0])*fs.Attitude.Pitch.Speed*fs.Attitude.Yaw.Speed
			-
			fs.SimParameters.PolarInertia * fs.Attitude.Pitch.Speed * (fs.Inputs.Speed.Front - fs.Inputs.Speed.Left + fs.Inputs.Speed.Back - fs.Inputs.Speed.Right)
			+
			fs.SimParameters.Length * (fs.Inputs.Force.Right - fs.Inputs.Force.Left)
		);
}

double AQD::Simulation::__Yaw(SState state, AQD::Simulation::SFullStateData fs)
{
	return 1 / fs.SimParameters.Inertia[2] *
		(
			(fs.SimParameters.Inertia[0] - fs.SimParameters.Inertia[1])*fs.Attitude.Pitch.Speed*fs.Attitude.Roll.Speed
			+
			(- fs.Inputs.Torque.Left + fs.Inputs.Torque.Front - fs.Inputs.Torque.Right + fs.Inputs.Torque.Back)
		);
}