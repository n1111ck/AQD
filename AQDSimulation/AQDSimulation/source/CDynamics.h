#ifndef CDYNAMICS_H
#define CDYNAMICS_H

#include "CStates.h"

namespace AQD
{
	namespace Simulation
	{
		// Dynamic Equations
		double __Longitude(AQD::Simulation::SFullStateData fs);
		double __Latitude(AQD::Simulation::SFullStateData fs);
		double __Altitude(AQD::Simulation::SFullStateData fs);
		double __Pitch(AQD::Simulation::SFullStateData fs);
		double __Roll(AQD::Simulation::SFullStateData fs);
		double __Yaw(AQD::Simulation::SFullStateData fs);
	}
}

#endif