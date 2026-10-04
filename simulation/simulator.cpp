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
    ThreadSafeQueue<SimulationData>& csvQueue)
{
    sort(
        variations.begin(),
        variations.end(),
        [](const Variation& a, const Variation& b)
        {
            return a.getTime() < b.getTime();
        }
    );

    size_t variationIdx = 0;

    int totalSteps =
        static_cast<int>(totalTime / dt) + 1;

    cout << "Starting MULTITHREADED simulation for "
         << totalTime
         << " seconds.."
         << endl;

    /*
     * MOTOR THREAD
     */
    thread motorThread([&]()
    {
        double currentTime = 0.0;

        // Send initial motor speed
        speedQueue.push(motor->getSpeed());

        for (int step = 0; step < totalSteps; step++)
        {
            double controlInput;

            if (!controlQueue.pop(controlInput))
                break;

            motor->updateSpeed(
                controlInput,
                dt
            );

            currentTime += dt;

            while (
                variationIdx < variations.size() &&
                currentTime >= variations[variationIdx].getTime()
            )
            {
                double loadChange =
                    variations[variationIdx].getLoadChange();

                double frictionChange =
                    variations[variationIdx].getFrictionChange();

                motor->addLoad(loadChange);
                motor->addFriction(frictionChange);

                variationIdx++;
            }

            speedQueue.push(
                motor->getSpeed()
            );
        }

        speedQueue.close();
    });


    /*
     * CONTROLLER THREAD
     */
    thread controllerThread([&]()
    {
        double currentTime = 0.0;

        for (int step = 0; step < totalSteps; step++)
        {
            double actualSpeed;

            if (!speedQueue.pop(actualSpeed))
                break;

            double error =
                comparator->getError(
                    target,
                    actualSpeed
                );

            double controlInput =
                controller->compute(
                    error,
                    dt
                );

            SimulationData data;

            data.time = currentTime;
            data.targetSpeed = target;
            data.actualSpeed = actualSpeed;
            data.error = error;
            data.controlInput = controlInput;

            result->write(
                data.time,
                data.targetSpeed,
                data.actualSpeed,
                data.error,
                data.controlInput
            );

            consoleQueue.push(data);
            csvQueue.push(data);

            controlQueue.push(controlInput);

            currentTime += dt;
        }

        controlQueue.close();
        consoleQueue.close();
        csvQueue.close();
    });
	
    controllerThread.join();
    motorThread.join();

    cout << "Multithreaded simulation complete."
         << endl;
}
