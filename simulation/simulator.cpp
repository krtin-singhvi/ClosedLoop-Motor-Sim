#include <iostream>
#include "simulator.h"
#include <stdexcept>
#include <algorithm>

using namespace std;

// Complete wiring of components
Simulator::Simulator(Motor* m, Controller* ctrl, Comparator* cmp, 
			     SimulationResult* r, double target_, double dt_, double time) : 
			     motor(m), controller(ctrl), comparator(cmp), result(r),
		             target(target_), dt(dt_), totalTime(time) 
{
	if(!m || !ctrl || !cmp || !r)
		throw invalid_argument("Simulator: Null component");
	if(dt <= 0.0)
		throw invalid_argument("Simulator: dt must be positive");
	if(totalTime <= 0.0)
		throw invalid_argument("Simulator: total time must be positive");
}

void Simulator::addVariation(const Variation& variation){
	variations.push_back(variation);
}

void Simulator::runSimulation() {
	double currentTime = 0.0;
	
	sort(variations.begin(), variations.end(),
	     [](const Variation& a, const Variation& b){
	     return a.getTime() < b.getTime();
	     });
	
	int variationIdx = 0;

	cout << "Starting simulation for " << totalTime << " seconds.." << endl;
	
	while(currentTime <= totalTime){
		// Process scheduled loads
		while(variationIdx < variations.size() && currentTime >= variations[variationIdx].getTime()){
			double loadChange = variations[variationIdx].getLoadChange();
			double frictionChange = variations[variationIdx].getFrictionChange();
			motor->addLoad(loadChange);
			motor->addFriction(frictionChange);

			cout << "[Time: " << currentTime << "s] ";
			if(loadChange != 0){
				cout << "Load changed by " << loadChange << " N.m ";
			}
			if(frictionChange != 0){
				cout << "Friction changed by " << frictionChange << " N.m.s/rad";
			}
			cout << endl;
			variationIdx++;
		}


		
		double actualSpeed = motor->getSpeed();	
	
		double error = comparator->getError(target, actualSpeed);	

		double controlInput = controller->compute(error, dt);		// Compute control effort

		result->write(currentTime, target, actualSpeed, error, controlInput);	

		motor->updateSpeed(controlInput, dt);	// Update for next timestamp
		currentTime += dt;
	}
	cout << "Simulation Complete.." << endl;
}
		
