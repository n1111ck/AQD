#include "CPlant.h"
#include "CProperties.h"
#include <math.h>
#include <iostream>

// Dynamic private methods
double AQD::CPlant::__Longitude(AQD::Prop::SInputs inputs, AQD::Prop::SState state)
{
	return (inputs.Force.Front + inputs.Force.Left + inputs.Force.Back + inputs.Force.Right) / this->m_Mass *
		(
			sin(this->m_Attitude.Yaw.Position) * sin(this->m_Attitude.Roll.Position)
			+
			cos(this->m_Attitude.Yaw.Position) * cos(this->m_Attitude.Roll.Position) * sin(this->m_Attitude.Pitch.Position)
		);
}
double AQD::CPlant::__Latitude(AQD::Prop::SInputs inputs, AQD::Prop::SState state)
{
	return (inputs.Force.Front + inputs.Force.Left + inputs.Force.Back + inputs.Force.Right) / this->m_Mass *
		(
			sin(this->m_Attitude.Yaw.Position) * cos(this->m_Attitude.Roll.Position) * sin(this->m_Attitude.Pitch.Position)
			-
			cos(this->m_Attitude.Yaw.Position) * sin(this->m_Attitude.Roll.Position)
		);
}
double AQD::CPlant::__Altitude(AQD::Prop::SInputs inputs, AQD::Prop::SState state)
{
	return (inputs.Force.Front + inputs.Force.Left + inputs.Force.Back + inputs.Force.Right) / this->m_Mass * cos(this->m_Attitude.Pitch.Position) * cos(this->m_Attitude.Roll.Position)
		-
		this->c_Gravity;
}
double AQD::CPlant::__Pitch(AQD::Prop::SInputs inputs, AQD::Prop::SState state)
{
	return 1 / this->m_Inertia.X *
		(
			(this->m_Inertia.Y - this->m_Inertia.Z)*this->m_Attitude.Roll.Speed* this->m_Attitude.Yaw.Speed
			-
			this->m_PolarInertia * this->m_Attitude.Roll.Speed * (inputs.Speed.Front - inputs.Speed.Left + inputs.Speed.Back - inputs.Speed.Right)
			+
			this->m_Length * (inputs.Force.Back - inputs.Force.Front)
		);
}
double AQD::CPlant::__Roll(AQD::Prop::SInputs inputs, AQD::Prop::SState state)
{
	return 1 / this->m_Inertia.Y *
		(
			(this->m_Inertia.Z - this->m_Inertia.X)*this->m_Attitude.Pitch.Speed* this->m_Attitude.Yaw.Speed
			-
			this->m_PolarInertia * this->m_Attitude.Pitch.Speed * (inputs.Speed.Front - inputs.Speed.Left + inputs.Speed.Back - inputs.Speed.Right)
			+
			this->m_Length * (inputs.Force.Right - inputs.Force.Left)
		);
}
double AQD::CPlant::__Yaw(AQD::Prop::SInputs inputs, AQD::Prop::SState state)
{
	return 1 / this->m_Inertia.Z *
		(
			(this->m_Inertia.X - this->m_Inertia.Y)*this->m_Attitude.Pitch.Speed* this->m_Attitude.Roll.Speed
			+
			(-inputs.Torque.Left + inputs.Torque.Front - inputs.Torque.Right + inputs.Torque.Back)
		);
}
double AQD::CPlant::__State(AQD::Prop::SInputs inputs, AQD::Prop::SState state, AQD::Prop::EState idState)
{
	switch (idState)
	{
		case AQD::Prop::Longitude:
			return this->__Longitude(inputs, state);
			break;
		case AQD::Prop::Latitude:
			return this->__Latitude(inputs, state);
			break;
		case AQD::Prop::Altitude:
			return this->__Altitude(inputs, state);
			break;
		case AQD::Prop::Pitch:
			return this->__Pitch(inputs, state);
			break;
		case AQD::Prop::Roll:
			return this->__Roll(inputs, state);
			break;
		case AQD::Prop::Yaw:
			return this->__Yaw(inputs, state);
			break;
		default:
			return -1;
			break;
	}
}
// Integration method
AQD::Prop::SState AQD::CPlant::rk4(AQD::Prop::SInputs inputs, AQD::Prop::SState state, AQD::Prop::EState idState)
{
	static double dv[5];
	static double dx[5];
	static double time = this->m_Time;
	struct AQD::Prop::SState nextState;

	nextState = state;
	nextState.Accel = this->__State(inputs, nextState, idState);
	dv[1] = this->m_Step * this->__State(inputs, nextState, idState);
	dx[1] = this->m_Step * nextState.Speed;

	time = this->m_Time + this->m_Step / 2;
	nextState.Speed = state.Speed + dv[1] / 2;
	nextState.Position = state.Position + dx[1] / 2;
	dv[2] = this->m_Step * this->__State(inputs, nextState, idState);
	dx[2] = this->m_Step * (state.Speed + dv[1] / 2);

	nextState.Speed = state.Speed + dv[2] / 2;
	nextState.Position = state.Position + dx[2] / 2;
	dv[3] = this->m_Step * this->__State(inputs, nextState, idState);
	dx[3] = this->m_Step * (state.Speed + dv[2] / 2);

	time = this->m_Time + this->m_Step;
	nextState.Speed = state.Speed + dv[3];
	nextState.Position = state.Position + dx[3];
	dv[4] = this->m_Step * this->__State(inputs, nextState, idState);
	dx[4] = this->m_Step * (nextState.Speed + dv[4]);

	dv[0] = (dv[1] + 2 * dv[2] + 2 * dv[3] + dv[4]) / 6;
	dx[0] = (dx[1] + 2 * dx[2] + 2 * dx[3] + dx[4]) / 6;

	nextState.Speed += dv[0];
	nextState.Position += dx[0];

	return nextState;
}
// Constructor and destructor
AQD::CPlant::CPlant(bool activeSimulation)
{
	this->m_ActiveSimulation = activeSimulation;

	struct AQD::Prop::SState nullState;
	nullState.Position = 0;
	nullState.Speed = 0;
	nullState.Accel = 0;

	// Ambient attributes
	this->m_WindForce = 0;
	this->m_WindDirection = 0;
	this->m_Time = 0;
	this->m_Step = 1e-1;
	// Structure attributes
	this->m_Mass = 2;
	this->m_Inertia.X = 1.49e-2;
	this->m_Inertia.Y = 1.53e-2;
	this->m_Inertia.Z = 5.32e-2;
	this->m_PolarInertia = 1e-6;
	this->m_Length = 30e-2;
	// Sensors data
	this->m_Attitude.Longitude = nullState;
	this->m_Attitude.Latitude = nullState;
	this->m_Attitude.Altitude = nullState;
	this->m_Attitude.Pitch = nullState;
	this->m_Attitude.Roll = nullState;
	this->m_Attitude.Yaw = nullState;
}
AQD::CPlant::~CPlant()
{

}
// Factory Methods
AQD::Prop::SAttitude AQD::CPlant::update(AQD::Prop::SInputs inputs)
{
	// Active Simulation
	if (this->m_ActiveSimulation) 
	{
		// Longitude
		this->m_Attitude.Longitude = rk4(inputs, this->m_Attitude.Longitude, AQD::Prop::Longitude);
		// Latitude
		this->m_Attitude.Latitude = rk4(inputs, this->m_Attitude.Latitude, AQD::Prop::Latitude);
		// Altitude
		this->m_Attitude.Altitude = rk4(inputs, this->m_Attitude.Altitude, AQD::Prop::Altitude);
		// Pitch
		this->m_Attitude.Pitch = rk4(inputs, this->m_Attitude.Pitch, AQD::Prop::Pitch);
		// Roll
		this->m_Attitude.Roll = rk4(inputs, this->m_Attitude.Roll, AQD::Prop::Roll);
		// Yaw
		this->m_Attitude.Yaw = rk4(inputs, this->m_Attitude.Yaw, AQD::Prop::Yaw);

		// Iterate time by a step of time
		this->m_Time += m_Step;

		return this->m_Attitude;
	}

	// Active Implementation
			
	return this->m_Attitude;
}
AQD::Prop::SAttitude AQD::CPlant::getAttitude()
{
	return this->m_Attitude;
}

// Setters
void AQD::CPlant::setWindForce(double windForce)
{
	this->m_WindForce = windForce;
}
void AQD::CPlant::setWindDirection(double windDirection)
{
	this->m_WindDirection = windDirection;
}
void AQD::CPlant::setMass(double mass)
{
	this->m_Mass = mass;
}
void AQD::CPlant::setInertia(AQD::Prop::SPropertyByAxis inertia)
{
	this->m_Inertia = inertia;
}
void AQD::CPlant::setPolarInertia(double polarInertia)
{
	this->m_PolarInertia = polarInertia;
}
void AQD::CPlant::setLength(double length)
{
	this->m_Length = length;
}
// Getters
double AQD::CPlant::getWindForce()
{
	return this->m_WindForce;
}
double AQD::CPlant::getWindDirection()
{
	return this->m_WindDirection;
}
double AQD::CPlant::getMass()
{
	return this->m_Mass;
}
AQD::Prop::SPropertyByAxis AQD::CPlant::getInertia()
{
	return this->m_Inertia;
}
double AQD::CPlant::getPolarInertia()
{
	return this->m_PolarInertia;
}
double AQD::CPlant::getLength()
{
	return this->m_Length;
}
