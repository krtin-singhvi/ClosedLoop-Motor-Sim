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
	
	size_t variationIdx = 0;

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

void Simulator::runMultiThreaded(
    ThreadSafeQueue<double>& controlQueue,
    ThreadSafeQueue<double>& speedQueue,
    ThreadSafeQueue<SimulationData>& consoleQueue,
    ThreadSafeQueue<SimulationData>& csvQueue
)
{
    double time = 0.0;
    size_t variationIdx = 0;

    while (time <= totalTime)
    {
        // Apply environmental variations
        while (variationIdx < variations.size() &&
               variations[variationIdx].getTime() <= time)
        {
            motor.applyVariation(variations[variationIdx]);
            variationIdx++;
        }

        // Get motor speed
        double actualSpeed = motor.getSpeed();

        // Send speed to controller
        speedQueue.push(actualSpeed);

        // Wait for controller output
        double controlInput;

        if (!controlQueue.pop(controlInput))
            break;

        // Apply controller output to motor
        motor.update(controlInput, dt);

        // Calculate error
        double error = comparator.compute(
            targetSpeed,
            actualSpeed
        );

        // Create simulation data
        SimulationData data;

        data.time = time;
        data.targetSpeed = targetSpeed;
        data.actualSpeed = actualSpeed;
        data.error = error;
        data.controlInput = controlInput;

        // Store result
        result.write(data);

        // Send data to output threads
        consoleQueue.push(data);
        csvQueue.push(data);

        time += dt;
    }

    // Tell output queues that no more data will arrive
    consoleQueue.close();
    csvQueue.close();
}
		
