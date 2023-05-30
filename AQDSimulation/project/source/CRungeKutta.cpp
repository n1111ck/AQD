#include "CRungeKutta.h"
#include "CDynamics.h"
#include "CStates.h"

AQD::Simulation::SState rk4(AQD::Simulation::SState state, AQD::Simulation::SFullStateData fs, double (*func)(AQD::Simulation::SState move, AQD::Simulation::SFullStateData))
{
	static double dv[5];
	static double dx[5];
	static double time;
	struct AQD::Simulation::SState stateBuffer;

	time = fs.SimParameters.Time;

	stateBuffer = state;
	stateBuffer.Accel = func(stateBuffer, fs);
	dv[1] = fs.SimParameters.Step * func(stateBuffer, fs);
	dx[1] = fs.SimParameters.Step * stateBuffer.Speed;

	fs.SimParameters.Time = time + fs.SimParameters.Step / 2;
	stateBuffer.Speed = state.Speed + dv[1] / 2;
	stateBuffer.Position = state.Position + dx[1] / 2;
	dv[2] = fs.SimParameters.Step * func(stateBuffer, fs);
	dx[2] = fs.SimParameters.Step * (state.Speed + dv[1] / 2);

	fsBuffer.SpeedData.Pitch = fs->SpeedData.Pitch + dv[2] / 2;
	fsBuffer.PositionData.Pitch = fs->PositionData.Pitch + dx[2] / 2;
	dv[3] = fs->SimParameters.Step * __Pitch(fsBuffer);
	dx[3] = fs->SimParameters.Step * (fs->SpeedData.Pitch + dv[2] / 2);

	fsBuffer.SimParameters.Time = fs->SimParameters.Time + fs->SimParameters.Step;
	fsBuffer.SpeedData.Pitch = fs->SpeedData.Pitch + dv[3];
	fsBuffer.PositionData.Pitch = fs->PositionData.Pitch + dx[3];
	dv[4] = fs->SimParameters.Step * __Pitch(fsBuffer);
	dx[4] = fs->SimParameters.Step * (fs->SpeedData.Pitch + dv[4]);

	dv[0] = (dv[1] + 2 * dv[2] + 2 * dv[3] + dv[4]) / 6;
	dx[0] = (dx[1] + 2 * dx[2] + 2 * dx[3] + dx[4]) / 6;

	fs->SpeedData.Pitch += dv[0];
	fs->PositionData.Pitch += dx[0];


}

void AQD::Simulation::simulate(AQD::Simulation::SFullStateData* fs)
{
	double dv[5];
	double dx[5];
	AQD::Simulation::SFullStateData fsBuffer;

	// Pitch
	fsBuffer = *fs;
	fs->AccelData.Pitch = __Pitch(fsBuffer);
	dv[1] = fs->SimParameters.Step * __Pitch(fsBuffer);
	dx[1] = fs->SimParameters.Step * fs->SpeedData.Pitch;

	fsBuffer.SimParameters.Time = fs->SimParameters.Time + fs->SimParameters.Step / 2;
	fsBuffer.SpeedData.Pitch = fs->SpeedData.Pitch + dv[1] / 2;
	fsBuffer.PositionData.Pitch = fs->PositionData.Pitch + dx[1] / 2;
	dv[2] = fs->SimParameters.Step * __Pitch(fsBuffer);
	dx[2] = fs->SimParameters.Step * (fs->SpeedData.Pitch + dv[1]/2);

	fsBuffer.SpeedData.Pitch = fs->SpeedData.Pitch + dv[2] / 2;
	fsBuffer.PositionData.Pitch = fs->PositionData.Pitch + dx[2] / 2;
	dv[3] = fs->SimParameters.Step * __Pitch(fsBuffer);
	dx[3] = fs->SimParameters.Step * (fs->SpeedData.Pitch + dv[2] / 2);

	fsBuffer.SimParameters.Time = fs->SimParameters.Time + fs->SimParameters.Step;
	fsBuffer.SpeedData.Pitch = fs->SpeedData.Pitch + dv[3];
	fsBuffer.PositionData.Pitch = fs->PositionData.Pitch + dx[3];
	dv[4] = fs->SimParameters.Step * __Pitch(fsBuffer);
	dx[4] = fs->SimParameters.Step * (fs->SpeedData.Pitch + dv[4]);

	dv[0] = (dv[1] + 2 * dv[2] + 2 * dv[3] + dv[4]) / 6;
	dx[0] = (dx[1] + 2 * dx[2] + 2 * dx[3] + dx[4]) / 6;

	fs->SpeedData.Pitch += dv[0];
	fs->PositionData.Pitch += dx[0];

	// Roll
	fsBuffer = *fs;
	fs->AccelData.Roll = __Roll(fsBuffer);
	dv[1] = fs->SimParameters.Step * __Roll(fsBuffer);
	dx[1] = fs->SimParameters.Step * fs->SpeedData.Roll;

	fsBuffer.SimParameters.Time = fs->SimParameters.Time + fs->SimParameters.Step / 2;
	fsBuffer.SpeedData.Roll = fs->SpeedData.Roll + dv[1] / 2;
	fsBuffer.PositionData.Roll = fs->PositionData.Roll + dx[1] / 2;
	dv[2] = fs->SimParameters.Step * __Roll(fsBuffer);
	dx[2] = fs->SimParameters.Step * (fs->SpeedData.Roll + dv[1] / 2);

	fsBuffer.SpeedData.Roll = fs->SpeedData.Roll + dv[2] / 2;
	fsBuffer.PositionData.Roll = fs->PositionData.Roll + dx[2] / 2;
	dv[3] = fs->SimParameters.Step * __Roll(fsBuffer);
	dx[3] = fs->SimParameters.Step * (fs->SpeedData.Roll + dv[2] / 2);

	fsBuffer.SimParameters.Time = fs->SimParameters.Time + fs->SimParameters.Step;
	fsBuffer.SpeedData.Roll = fs->SpeedData.Roll + dv[3];
	fsBuffer.PositionData.Roll = fs->PositionData.Roll + dx[3];
	dv[4] = fs->SimParameters.Step * __Roll(fsBuffer);
	dx[4] = fs->SimParameters.Step * (fs->SpeedData.Roll + dv[4]);

	dv[0] = (dv[1] + 2 * dv[2] + 2 * dv[3] + dv[4]) / 6;
	dx[0] = (dx[1] + 2 * dx[2] + 2 * dx[3] + dx[4]) / 6;

	fs->SpeedData.Roll += dv[0];
	fs->PositionData.Roll += dx[0];

	// Yaw
	fsBuffer = *fs;
	fs->AccelData.Yaw = __Yaw(fsBuffer);
	dv[1] = fs->SimParameters.Step * __Yaw(fsBuffer);
	dx[1] = fs->SimParameters.Step * fs->SpeedData.Yaw;

	fsBuffer.SimParameters.Time = fs->SimParameters.Time + fs->SimParameters.Step / 2;
	fsBuffer.SpeedData.Yaw = fs->SpeedData.Yaw + dv[1] / 2;
	fsBuffer.PositionData.Yaw = fs->PositionData.Yaw + dx[1] / 2;
	dv[2] = fs->SimParameters.Step * __Yaw(fsBuffer);
	dx[2] = fs->SimParameters.Step * (fs->SpeedData.Yaw + dv[1] / 2);

	fsBuffer.SpeedData.Yaw = fs->SpeedData.Yaw + dv[2] / 2;
	fsBuffer.PositionData.Yaw = fs->PositionData.Yaw + dx[2] / 2;
	dv[3] = fs->SimParameters.Step * __Yaw(fsBuffer);
	dx[3] = fs->SimParameters.Step * (fs->SpeedData.Yaw + dv[2] / 2);

	fsBuffer.SimParameters.Time = fs->SimParameters.Time + fs->SimParameters.Step;
	fsBuffer.SpeedData.Yaw = fs->SpeedData.Yaw + dv[3];
	fsBuffer.PositionData.Yaw = fs->PositionData.Yaw + dx[3];
	dv[4] = fs->SimParameters.Step * __Yaw(fsBuffer);
	dx[4] = fs->SimParameters.Step * (fs->SpeedData.Yaw + dv[4]);

	dv[0] = (dv[1] + 2 * dv[2] + 2 * dv[3] + dv[4]) / 6;
	dx[0] = (dx[1] + 2 * dx[2] + 2 * dx[3] + dx[4]) / 6;

	fs->SpeedData.Yaw += dv[0];
	fs->PositionData.Yaw += dx[0];
	
	fs->SimParameters.Time += fs->SimParameters.Step;
}
