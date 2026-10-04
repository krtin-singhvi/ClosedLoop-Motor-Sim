#pragma once
#include <vector>
#include <thread>
#include "motor.h"
#include "components.h"
#include "controller.h"
#include "simulationresult.h"
#include "variation.h"
#include "threadSafeQueue.h"

class Simulator{
private:
	Motor* motor;
	Controller* controller;
	Comparator* comparator;
	SimulationResult* result;
	double target, dt, totalTime;
	std::vector<Variation> variations;

public:
	Simulator(Motor* m, Controller* ctrl, Comparator *cmp,
		      SimulationResult *r, double target, double dt, double time);
	void addVariation(const Variation& variation);
	void runSimulation();
	void runMultiThreaded(
		ThreadSafeQueue<double>& controlQueue,
		ThreadSafeQueue<double>& speedQueue,
		ThreadSafeQueue<SimulationData>& consoleQueue,
		ThreadSafeQueue<SimulationData>& csvQueue);

};
	
