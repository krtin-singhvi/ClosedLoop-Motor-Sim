#include <iostream>
#include "simulator.h"
#include <stdexcept>
using namespace std;

double Comparator::getError(double targetSpeed, double actualSpeed) const{
        return targetSpeed - actualSpeed;
}

// Complete wiring of components
Simulator::Simulator(Motor* m, PIDController* ctrl, Comparator* cmp, 
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

// To introduce load at any given time
void Simulator::scheduleLoad(double atTime, double load){
	loadSchedule.push_back({atTime, load});
}

void Simulator::scheduleAllLoads(const vector<LoadEvent>& loads){
	loadSchedule.insert(loadSchedule.end(), loads.begin(), loads.end());
}

void Simulator::scheduleFriction(double atTime, double friction){
	frictionSchedule.push_back({atTime, friction});
}

void Simulator::scheduleAllFriction(const vector<FrictionEvent>& frictions){
	frictionSchedule.insert(frictionSchedule.end(), frictions.begin(), frictions.end());
}

void Simulator::runSimulation() {
	double currentTime = 0.0;

	vector<bool> loadApplied(loadSchedule.size(), false);
	vector<bool> frictionApplied(frictionSchedule.size(), false);

	cout << "Starting simulation for " << totalTime << " seconds.." << endl;
	
	while(currentTime <= totalTime){
		// Process scheduled loads
		for(int i = 0; i < loadSchedule.size(); i++){
			if(!loadApplied[i] && currentTime >= loadSchedule[i].time){	
				motor->setLoad(loadSchedule[i].load);
				loadApplied[i] = true;
				cout << "[Time: " << currentTime << "s] Applied Load Torque = " << loadSchedule[i].load << " N.m" << endl;
			}
		}

		// Process scheduled friction changes
		for(int i = 0; i < frictionSchedule.size(); i++){
			if(!frictionApplied[i] && currentTime >= frictionSchedule[i].time){
				motor->setFriction(frictionSchedule[i].friction);
				frictionApplied[i] = true;
				cout << "[Time: " << currentTime << "s] Applied Friction = " << frictionSchedule[i].friction << " N.m.s/rad" << endl;
			}
		}	

		
		double actualSpeed = motor->getSpeed();	
	
		double error = comparator->getError(target, actualSpeed);	

		double controlInput = controller->compute(error, dt);		// Compute control effort

		result->write(currentTime, target, actualSpeed, error, controlInput);	

		motor->update(controlInput, dt);	// Update for next timestamp
		currentTime += dt;
	}
	cout << "Simulation Complete.." << endl;
}
		
