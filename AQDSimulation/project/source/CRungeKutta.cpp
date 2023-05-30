#include "CRungeKutta.h"
#include "CDynamics.h"
#include "CStates.h"

AQD::Simulation::SState rk4(AQD::Simulation::SState state, AQD::Simulation::SFullStateData fs, double (*func)(AQD::Simulation::SState move, AQD::Simulation::SFullStateData))
{
	static double dv[5];
	static double dx[5];
	static double time;
	struct AQD::Simulation::SState nextState;

	time = fs.SimParameters.Time;

	nextState = state;
	nextState.Accel = func(nextState, fs);
	dv[1] = fs.SimParameters.Step * func(nextState, fs);
	dx[1] = fs.SimParameters.Step * nextState.Speed;

	fs.SimParameters.Time = time + fs.SimParameters.Step / 2;
	nextState.Speed = state.Speed + dv[1] / 2;
	nextState.Position = state.Position + dx[1] / 2;
	dv[2] = fs.SimParameters.Step * func(nextState, fs);
	dx[2] = fs.SimParameters.Step * (state.Speed + dv[1] / 2);

	nextState.Speed = state.Speed + dv[2] / 2;
	nextState.Position = state.Position + dx[2] / 2;
	dv[3] = fs.SimParameters.Step * func(nextState, fs);
	dx[3] = fs.SimParameters.Step * (state.Speed + dv[2] / 2);

	fs.SimParameters.Time = time + fs.SimParameters.Step;
	nextState.Speed = state.Speed + dv[3];
	nextState.Position = state.Position + dx[3];
	dv[4] = fs.SimParameters.Step * func(nextState, fs);
	dx[4] = fs.SimParameters.Step * (nextState.Speed + dv[4]);

	dv[0] = (dv[1] + 2 * dv[2] + 2 * dv[3] + dv[4]) / 6;
	dx[0] = (dx[1] + 2 * dx[2] + 2 * dx[3] + dx[4]) / 6;

	nextState.Speed += dv[0];
	nextState.Position += dx[0];

	return nextState;
}

void AQD::Simulation::simulate(AQD::Simulation::SFullStateData* fs)
{
	// Longitude
	fs->Attitude.Longitude = rk4(fs->Attitude.Longitude, *fs, &__Longitude);
	// Latitude
	fs->Attitude.Latitude = rk4(fs->Attitude.Latitude, *fs, &__Latitude);
	// Altitude
	fs->Attitude.Altitude = rk4(fs->Attitude.Altitude, *fs, &__Altitude);
	// Pitch
	fs->Attitude.Pitch = rk4(fs->Attitude.Pitch, *fs, &__Pitch);
	// Roll
	fs->Attitude.Roll = rk4(fs->Attitude.Roll, *fs, &__Roll);
	// Yaw
	fs->Attitude.Yaw = rk4(fs->Attitude.Yaw, *fs, &__Yaw);

	// Iterate time by a step of time
	fs->SimParameters.Time += fs->SimParameters.Step;
}
