#ifndef CMOTOR_H
#define CMOTOR_H

#include "CProperties.h"

namespace AQD
{
	class CMotor
	{
		private:
			double Kv;	// Constant that relates voltage with motor speed (linear)
			double Kf;	// Constant that relates motor speed with thrust (quadratic)
			double Kt;	// Constant that relates motor speed with torque (quadratic)
		public:
			CMotor(double Kv, double Kf, double Kt);
			~CMotor();
			AQD::Prop::SInputs convert(AQD::Prop::SPropertyByPropeller mVoltage);
	};
}

#endif