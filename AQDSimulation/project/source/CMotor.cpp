#include "CMotor.h"
#include "CProperties.h"

#include <math.h>

// Constructor and destructor
AQD::CMotor::CMotor(double Kv, double Kf, double Kt)
{
	this->Kf = Kf;
	this->Kt = Kt;
	this->Kv = Kv;
}
AQD::CMotor::~CMotor()
{

}

AQD::Prop::SInputs AQD::CMotor::convert(AQD::Prop::SPropertyByPropeller mVoltage)
{
	AQD::Prop::SInputs inputs;

	inputs.Speed.Front = this->Kv * mVoltage.Front;
	inputs.Speed.Left = this->Kv * mVoltage.Left;
	inputs.Speed.Back = this->Kv * mVoltage.Back;
	inputs.Speed.Right = this->Kv * mVoltage.Right;

	inputs.Force.Front = this->Kf * pow(inputs.Speed.Front, 2);
	inputs.Force.Left = this->Kf * pow(inputs.Speed.Left, 2);
	inputs.Force.Back = this->Kf * pow(inputs.Speed.Back, 2);
	inputs.Force.Right = this->Kf * pow(inputs.Speed.Right, 2);

	inputs.Torque.Front = this->Kt * pow(inputs.Speed.Front, 2);
	inputs.Torque.Left = this->Kt * pow(inputs.Speed.Left, 2);
	inputs.Torque.Back = this->Kt * pow(inputs.Speed.Back, 2);
	inputs.Torque.Right = this->Kt * pow(inputs.Speed.Right, 2);

	return inputs;
}