// AADSimulation.cpp : Defines the entry point for the application.
//


#include "CSimulation.h"
#include "CDynamics.h"
#include "CMotor.h"

#include <cstdlib>
#include <iostream>

int main()
{
	struct AQD::Prop::SFullStateData* fs = nullptr;
	struct AQD::Prop::SMotor* motorConst = nullptr;
	struct AQD::Prop::SPropertyByPropeller voltage;

	// Simulation Pamaters
	fs->SimParameters.Mass = 2;
	fs->SimParameters.Inertia[0] = 1.49e-2;
	fs->SimParameters.Inertia[1] = 1.53e-2;
	fs->SimParameters.Inertia[2] = 5.32e-2;
	fs->SimParameters.Length = 30e-2;
	fs->SimParameters.WindDirection = 0;
	fs->SimParameters.PolarInertia = 1e-8;
	fs->SimParameters.WindForce = 0;
	fs->SimParameters.Step = 1e-4;
	fs->SimParameters.Time = 0;

	// Motor experimental values
	motorConst->Kv = 110;
	motorConst->Kf = 1.47;
	motorConst->Kt = 0.007;

	voltage.Front = 4;
	voltage.Left = 5;
	voltage.Right = 5;
	voltage.Back = 6;

	fs->Inputs = AQD::Motor::convert(*motorConst, voltage);

	fs->Attitude.Longitude = { 0 };
	fs->Attitude.Latitude = { 0 };
	fs->Attitude.Altitude = { 0 };
	fs->Attitude.Pitch = { 0 };
	fs->Attitude.Roll = { 0 };
	fs->Attitude.Yaw = { 0 };
	

	for (int i = 0; i < 10; i++)
	{
		AQD::Simulation::simulate(fs);
		std::cout << fs->Attitude.Pitch.Position << " " << fs->Attitude.Pitch.Speed << " " << fs->Attitude.Pitch.Accel << std::endl;
	}

	return 0;
}
