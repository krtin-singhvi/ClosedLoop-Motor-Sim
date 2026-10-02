#pragma once
#include <vector>
#include "motor.h"
#include "controller.h"
#include "simulationResult.h"

struct LoadEvent{
	double time;
	double load;
};

struct FrictionEvent{
	double time;
	double friction;
};

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
	std::vector<LoadEvent> loadSchedule;
	std::vector<FrictionEvent> frictionSchedule;

public:
	Simulator(Motor* m, Controller* ctrl, Comparator *cmp,
		      SimulationResult *r, double target, double dt, double time);
	
	void scheduleLoad(double atTime, double load);
	void scheduleFriction(double atTime, double friction);
	
	void scheduleAllLoads(const std::vector<LoadEvent>& loads);
	void scheduleAllFriction(const std::vector<FrictionEvent>& frictions);
	void runSimulation();
};
	
