#pragma once
#include "motor.h"
#include "controller.h"
#include "simulationResult.h"

class Comparator{
public:
        double getError(double targetSpeed, double actualSpeed) const;
};

class Simulator{
private:
	Motor* motor;
	Controller* controller;
	Comparator* comparator;
	SimulationResult* result;
	double target, dt, totalTime;
	double loadTime, loadValue;
	bool loadScheduled;

public:
	Simulator(Motor* m, Controller* ctrl, Comparator *cmp,
		      SimulationResult *r, double target, double dt, double time);

	void scheduleLoad(double atTime, double load);
	void runSimulation();
};
	
