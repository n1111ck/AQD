// AADSimulation.cpp : Defines the entry point for the application.
//


#include "CRungeKutta.h"
#include "CDynamics.h"

#include <cstdlib>
#include <iostream>

int main()
{
	struct AQD::Simulation::SFullStateData* fs;
	struct AQD::Simulation::SGeneralStateData gsd;
	struct AQD::Simulation::SPropertyByPropeller pbp;

	fs = (AQD::Simulation::SFullStateData*) malloc(sizeof(AQD::Simulation::SFullStateData));



	for(int i = 0; i < 3; i++)
		fs->SimParameters.Inertia[i] = 1;
	fs->SimParameters.Length = 1;
	fs->SimParameters.Mass = 1;
	fs->SimParameters.WindDirection = 1;
	fs->SimParameters.WindForce = 1;
	fs->SimParameters.Step = 1e-4;
	fs->SimParameters.Time = 0;

	pbp.Front = 1;
	pbp.Back = 1;
	pbp.Left = 1;
	pbp.Right = 1;
	fs->Inputs.Force = pbp;
	fs->Inputs.Force.Front = 0.5;
	fs->Inputs.Force.Back = 1.5;
	fs->Inputs.Torque = pbp;

	gsd.Altitude = 0;
	gsd.Latitude = 0;
	gsd.Longitude = 0;
	gsd.Pitch = 0;
	gsd.Roll = 0;
	gsd.Yaw = 0;
	fs->PositionData = gsd;
	fs->SpeedData = gsd;
	fs->AccelData = gsd;

	std::cout << AQD::Simulation::__Pitch(*fs);

	for (int i = 0; i < 10; i++)
	{
		AQD::Simulation::simulate(fs);
		std::cout << fs->AccelData.Pitch << " " << fs->SpeedData.Pitch << " " << fs->PositionData.Pitch << std::endl;
	}

	return 0;
}
