#ifndef CPLANT_H
#define CPLANT_H

#include "CProperties.h"

namespace AQD 
{
	class CPlant
	{
		private:
			// Ambient attributes
			double m_WindForce;
			double m_WindDirection;
			const double c_Gravity = 9.81;
			// Simulation attributes
			double m_Time;
			double m_Step;
			bool m_ActiveSimulation;
			// Structure attributes
			double m_Mass;
			AQD::Prop::SPropertyByAxis m_Inertia;
			double m_PolarInertia;
			double m_Length;
			// Sensors data
			AQD::Prop::SAttitude m_Attitude;

			// Dynamic private methods
			double __Longitude(AQD::Prop::SInputs inputs, AQD::Prop::SState state);
			double __Latitude(AQD::Prop::SInputs inputs, AQD::Prop::SState state);
			double __Altitude(AQD::Prop::SInputs inputs, AQD::Prop::SState state);
			double __Pitch(AQD::Prop::SInputs inputs, AQD::Prop::SState state);
			double __Roll(AQD::Prop::SInputs inputs, AQD::Prop::SState state);
			double __Yaw(AQD::Prop::SInputs inputs, AQD::Prop::SState state);
			double __State(AQD::Prop::SInputs inputs, AQD::Prop::SState state, AQD::Prop::EState idState);

			// Integration Method
			AQD::Prop::SState rk4(AQD::Prop::SInputs inputs, AQD::Prop::SState state, AQD::Prop::EState idState);
			
		public:
			CPlant(bool activeSimulation);
			~CPlant();

			// Factory functions
			AQD::Prop::SAttitude update(AQD::Prop::SInputs inputs);
			AQD::Prop::SAttitude getAttitude();
				
			// Setters
			void setWindForce(double windForce);
			void setWindDirection(double windDirection);
			void setMass(double mass);
			void setInertia(AQD::Prop::SPropertyByAxis inertia);
			void setPolarInertia(double polarInertia);
			void setLength(double length);

			// Getters
			double getWindForce();
			double getWindDirection();
			double getMass();
			AQD::Prop::SPropertyByAxis getInertia();
			double getPolarInertia();
			double getLength();
	};
}

#endif

