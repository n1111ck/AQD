#ifndef CDYNAMICS_H
#define CDYNAMICS_H

#include "CStates.h"

namespace AQD
{
	namespace Simulation
	{
		// Dynamic Equations
		double __Longitude(SState state, AQD::Simulation::SFullStateData fs);
		double __Latitude(SState state, AQD::Simulation::SFullStateData fs);
		double __Altitude(SState state, AQD::Simulation::SFullStateData fs);
		double __Pitch(SState state, AQD::Simulation::SFullStateData fs);
		double __Roll(SState state, AQD::Simulation::SFullStateData fs);
		double __Yaw(SState state, AQD::Simulation::SFullStateData fs);
	}
}

#endif