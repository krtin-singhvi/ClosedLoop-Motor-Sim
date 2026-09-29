#include <iostream>
#include "controlSystem.h"
#include <stdexcept>
using namespace std;

// Complete wiring of components
ControlSystem::ControlSystem(Motor* m, Controller* ctrl, Comparator* cmp, 
			     SimulationResult* r, double target_, double dt_, double time) : 
			     motor(m), controller(ctrl), comparator(cmp), result(r),
		             target(target_), dt(dt_), totalTime(time), loadTime(0.0), loadValue(0.0), loadScheduled(false) 
{
	if(!m || !ctrl ||  !cmp || !r)
		throw invalid_argument("ControlSystem: Null component");
	if(dt <= 0.0)
		throw invalid_argument("ControlSystem: dt must be positive");
	if(totalTime <= 0.0)
		throw invalid_argument("ControlSystem: total time must be positive");
}

// To introduce load at any given time
void ControlSystem::scheduleLoad(double atTime, double load){
	loadTime = atTime;
	loadValue = load;
	loadScheduled = true;
}

void ControlSystem::runSimulation() {
	double currentTime = 0.0;
	
	cout << "Starting simulation for " << totalTime << " seconds.." << endl;
	
	while(currentTime <= totalTime){
		if(loadScheduled && currentTime >= loadTime){	// Check for scheduled load disturbance
			motor->setLoad(loadValue);
			loadScheduled = false;
		}
		
		double actualSpeed = motor->getSpeed();	
	
		double error = comparator->getError(target, actualSpeed);	

		double controlInput = controller->calculate(error, dt);		// Compute control effort

		result->write(currentTime, target, actualSpeed, error, controlInput);	

		motor->update(controlInput, dt);	// Update for next timestamp

		currentTime += dt;
	}
	cout << "Simulation Complete.." << endl;
}
		



