#pragma once
#include <vector>
#include "motor.h"
#include "pid_controller.h"
#include "simulationResult.h"
#include "variation.h"

class Comparator{
public:
        double getError(double targetSpeed, double actualSpeed) const;
};

class Simulator{
private:
	Motor* motor;
	PIDController* controller;
	Comparator* comparator;
	SimulationResult* result;
	double target, dt, totalTime;
	std::vector<Variation> variations;

public:
	Simulator(Motor* m, PIDController* ctrl, Comparator *cmp,
		      SimulationResult *r, double target, double dt, double time);
	void addVariation(const Variation& variation);
	void runSimulation();
};
	
