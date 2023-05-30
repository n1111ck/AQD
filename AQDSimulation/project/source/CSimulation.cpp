#include "CSimulation.h"
#include "CDynamics.h"
#include "CProperties.h"

class CPlantSimulated
{
	private:
		// Ambient attributes
		double m_WindForce;
		double m_WindDirection;
		const double c_Gravity = 9.8067;
		// Simulation attributes
		double m_Time;
		double m_Step;
		// Structure attributes
		double m_Mass;
		AQD::Prop::SPropertyByAxis m_Inertia;
		double m_PolarInertia;
		double m_Length;
		// Sensors data
		AQD::Prop::SAttitude attitude;

		// Dynamic private methods
		double __Longitude(AQD::Prop::SState state)
		{
			return (fs.Inputs.Force.Front + fs.Inputs.Force.Left + fs.Inputs.Force.Back + fs.Inputs.Force.Right) / fs.SimParameters.Mass *
				(
					std::sin(fs.Attitude.Yaw.Position) * std::sin(fs.Attitude.Roll.Position)
					+
					std::cos(fs.Attitude.Yaw.Position) * std::cos(fs.Attitude.Roll.Position) * std::sin(fs.Attitude.Pitch.Position)
				);
		}
		double __Latitude(AQD::Prop::SState state)
		{
			return (fs.Inputs.Force.Front + fs.Inputs.Force.Left + fs.Inputs.Force.Back + fs.Inputs.Force.Right) / fs.SimParameters.Mass *
				(
					std::sin(fs.Attitude.Yaw.Position) * std::cos(fs.Attitude.Roll.Position) * std::sin(fs.Attitude.Pitch.Position)
					-
					std::cos(fs.Attitude.Yaw.Position) * std::sin(fs.Attitude.Roll.Position)
				);
		}
		double __Altitude(AQD::Prop::SState state)
		{
			return (fs.Inputs.Force.Front + fs.Inputs.Force.Left + fs.Inputs.Force.Back + fs.Inputs.Force.Right) / fs.SimParameters.Mass * std::cos(fs.Attitude.Pitch.Position) * std::cos(fs.Attitude.Roll.Position)
				-
				c_Gravity;
		}
		double __Pitch(AQD::Prop::SState state)
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
		double __Roll(AQD::Prop::SState state)
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
		double __Yaw(AQD::Prop::SState state)
		{
			return 1 / fs.SimParameters.Inertia[2] *
				(
				(fs.SimParameters.Inertia[0] - fs.SimParameters.Inertia[1])*fs.Attitude.Pitch.Speed*fs.Attitude.Roll.Speed
					+
					(-fs.Inputs.Torque.Left + fs.Inputs.Torque.Front - fs.Inputs.Torque.Right + fs.Inputs.Torque.Back)
				);
		}
		// Integration method
		void rk4() 
		{
			static double dv[5];
			static double dx[5];
			static double time;
			struct AQD::Prop::SState nextState;

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

	public:
		CPlantSimulated()
		{

		}
		~CPlantSimulated()
		{

		}
		void simulate()
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
};
