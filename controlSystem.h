#pragma once
#include "motor.h"
#include "controller.h"
#include "comparator.h"
#include "simulationResult.h"

class ControlSystem{
private:
	Motor* motor;
	Controller* controller;
	Comparator* comparator;
	SimulationResult* result;
	double target, dt, totalTime;
	double loadTime, loadValue;
	bool loadScheduled;

public:
	ControlSystem(Motor* m, Controller* ctrl, Comparator *cmp,
		      SimulationResult *r, double target, double dt, double time);

	void scheduleLoad(double atTime, double load);
	void runSimulation();
};
	
