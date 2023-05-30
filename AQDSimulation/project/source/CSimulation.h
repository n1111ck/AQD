#ifndef CRUNGEKUTTA_H
#define CRUNGEKUTTA_H

#include "CProperties.h"

namespace AQD 
{
	namespace Simulation
	{
		AQD::Prop::SState rk4(AQD::Prop::SState state, AQD::Prop::SFullStateData fs, double (*func)(AQD::Prop::SState state, AQD::Prop::SFullStateData));
		void simulate(AQD::Prop::SFullStateData* fs);
	}
}

#endif

