// AADSimulation.cpp : Defines the entry point for the application.
//


#include "CPlant.h"
#include "CMotor.h"

#include <cstdlib>
#include <iostream>

int main()
{
	struct AQD::Prop::SPropertyByPropeller v1, v2, v3;
	
	AQD::CMotor* motor = new AQD::CMotor(110.0, 5.07e-6, 7e-9);
	AQD::CPlant* plant = new AQD::CPlant(true);

	v1.Front = 8.94;
	v1.Left = 8.94;
	v1.Right = 8.94;
	v1.Back = 8.94;

	v2.Front = 9;
	v2.Left = 10;
	v2.Right = 10;
	v2.Back = 11;

	v3.Front = 11;
	v3.Left = 10;
	v3.Right = 10;
	v3.Back = 9;
	
	for (int i = 0; i < 100; i++)
	{
		std::cout << plant->getAttitude().Pitch.Position << std::endl;
		if(i < 10)
			plant->update(motor->convert(v1));
		else if(i < 11)
			plant->update(motor->convert(v2));
		else if(i < 12)
			plant->update(motor->convert(v3));
		else
			plant->update(motor->convert(v1));
	}

	return 0;
}
