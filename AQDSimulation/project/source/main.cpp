// AADSimulation.cpp : Defines the entry point for the application.
//


#include "CRungeKutta.h"
#include "CDynamics.h"

#include <cstdlib>
#include <iostream>

int main()
{
	struct AQD::Simulation::SFullStateData* fs = nullptr;


	fs->SimParameters.Mass = 2;
	fs->SimParameters.Inertia[0] = 0.09;
	fs->SimParameters.Inertia[1] = 0.11;
	fs->SimParameters.Inertia[2] = 0.20;
	fs->SimParameters.Length = 30e-2;
	fs->SimParameters.WindDirection = 0;
	fs->SimParameters.PolarInertia = 1e-8;
	fs->SimParameters.WindForce = 0;
	fs->SimParameters.Step = 1e-4;
	fs->SimParameters.Time = 0;

	fs->Inputs.Speed.Front = 2.45;
	fs->Inputs.Speed.Left = 2.5;
	fs->Inputs.Speed.Right = 2.5;
	fs->Inputs.Speed.Back = 2.65;

	fs->Inputs.Force.Front = 1.47 * pow(fs->Inputs.Force.Front,2);
	fs->Inputs.Force.Left = 1.47 * pow(fs->Inputs.Force.Left, 2);
	fs->Inputs.Force.Right = 1.47 * pow(fs->Inputs.Force.Right, 2);
	fs->Inputs.Force.Back = 1.47 * pow(fs->Inputs.Force.Back, 2);

	fs->Inputs.Force.Front = 0.7 * pow(fs->Inputs.Force.Front, 2);
	fs->Inputs.Force.Left = 0.7 * pow(fs->Inputs.Force.Left, 2);
	fs->Inputs.Force.Right = 0.7 * pow(fs->Inputs.Force.Right, 2);
	fs->Inputs.Force.Back = 0.7 * pow(fs->Inputs.Force.Back, 2);

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
