#ifndef CMOTOR_H
#define CMOTOR_H

#include "CProperties.h"

namespace AQD
{
	namespace Motor
	{
		AQD::Prop::SInputs convert(AQD::Prop::SMotor motor, AQD::Prop::SPropertyByPropeller mSpeed);
	}
}

#endif