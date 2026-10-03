#pragma once
#include <vector>
#include "motor.h"
#include "components.h"
#include "controller.h"
#include "simulationresult.h"
#include "variation.h"

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
};
	
