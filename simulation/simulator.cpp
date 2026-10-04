#include <iostream>
#include "simulator.h"
#include "variation.h"
#include <stdexcept>
#include <algorithm>

using namespace std;

// Complete wiring of components
Simulator::Simulator(double _Kp,
    double _Ki,
    double _Kd,
    double maxv,
    double minv,
    double j,
    double b,
    double k,
    double _target,
    double _dt,
    double _time): 
        motor(j, b, k),
        controller(_Kp, _Ki, _Kd, maxv, minv),
        comparator(),
        result(),
        variations(),
        dt(_dt),
        target(_target),
        totalTime(_time)
{
	if(dt <= 0.0)
		throw invalid_argument("Simulator: dt must be positive");
	if(totalTime <= 0.0)
		throw invalid_argument("Simulator: total time must be positive");
    if(target <= 0.0)
        throw invalid_argument("Simulator: target speed must be positive");
}

void Simulator::addVariation(){
        int numberOfVariations;

        cout << "Enter number of variations: ";
        cin >> numberOfVariations;

        if(numberOfVariations )

        for (int i = 0; i < numberOfVariations; i++)
        {
            double time;
            double loadChange;
            double frictionChange;

            cout << "\nVariation " << i + 1 << endl;

            cout << "Enter time (seconds): ";
            cin >> time;

            cout << "Enter load change: ";
            cin >> loadChange;

            cout << "Enter friction change: ";
            cin >> frictionChange;

            variations.push_back(Variation(time, loadChange, frictionChange));
        }
	
}

void Simulator::runSimulation() {
	double currentTime = 0.0;
	
	sort(variations.begin(), variations.end(),
	     [](const Variation& a, const Variation& b){
	     return a.getTime() < b.getTime();
	     });
	
	size_t variationIdx = 0;

	int totalSteps = static_cast<int>(totalTime / dt) + 1;

	cout << "Starting simulation for " << totalTime << " seconds.." << endl;
	
	for(int step = 0; step < totalSteps; step++){
		// Process scheduled loads
		while(variationIdx < variations.size() && currentTime >= variations[variationIdx].getTime()){
			double loadChange = variations[variationIdx].getLoadChange();
			double frictionChange = variations[variationIdx].getFrictionChange();
			motor.addLoad(loadChange);
			motor.addFriction(frictionChange);

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
		
		double actualSpeed = motor.getSpeed();	
	
		double error = comparator.getError(target, actualSpeed);	

		double controlInput = controller.compute(error, dt);		// Compute control effort

		result.write(currentTime, target, actualSpeed, error, controlInput);	

		motor.updateSpeed(controlInput, dt);	// Update for next timestamp
		currentTime += dt;
	}
	cout << "Simulation Complete.." << endl;
}

SimulationResult Simulator::returnResult() const{
    return result;
}
