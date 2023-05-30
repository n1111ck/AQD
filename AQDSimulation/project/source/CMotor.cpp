#include "CMotor.h"
#include "CProperties.h"

#include <cstdlib>
#include <iostream>

AQD::Prop::SInputs AQD::Motor::convert(AQD::Prop::SMotor mProperties, AQD::Prop::SPropertyByPropeller mVoltage)
{
	AQD::Prop::SInputs inputs;

	inputs.Speed.Front = mProperties.Kv*mVoltage.Front;
	inputs.Speed.Left = mProperties.Kv*mVoltage.Left;
	inputs.Speed.Back = mProperties.Kv*mVoltage.Back;
	inputs.Speed.Right = mProperties.Kv*mVoltage.Right;

	inputs.Force.Front = mProperties.Kf*pow(inputs.Speed.Front, 2);
	inputs.Force.Left = mProperties.Kf*pow(inputs.Speed.Left, 2);
	inputs.Force.Back = mProperties.Kf*pow(inputs.Speed.Back, 2);
	inputs.Force.Right = mProperties.Kf*pow(inputs.Speed.Right, 2);

	inputs.Torque.Front = mProperties.Kt*pow(inputs.Speed.Front, 2);
	inputs.Torque.Left = mProperties.Kt*pow(inputs.Speed.Left, 2);
	inputs.Torque.Back = mProperties.Kt*pow(inputs.Speed.Back, 2);
	inputs.Torque.Right = mProperties.Kt*pow(inputs.Speed.Right, 2);

	return inputs;
}