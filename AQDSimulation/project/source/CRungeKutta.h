#ifndef CRUNGEKUTTA_H
#define CRUNGEKUTTA_H

#include "CStates.h"

namespace AQD 
{
	namespace Simulation
	{
		AQD::Simulation::SState rk4(AQD::Simulation::SState state, AQD::Simulation::SFullStateData fs, double (*func)(AQD::Simulation::SState move, AQD::Simulation::SFullStateData));
		void simulate(AQD::Simulation::SFullStateData* fs);
	}
}

#endif

