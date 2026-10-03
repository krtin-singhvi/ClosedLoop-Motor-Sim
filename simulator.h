#pragma once
#include <vector>
#include "motor.h"
#include "pid_controller.h"
#include "simulationresult.h"
using namespace std;

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
	PIDController* controller;
	Comparator* comparator;
	SimulationResult* result;
	double target, dt, totalTime;
	vector<LoadEvent> loadSchedule;
	vector<FrictionEvent> frictionSchedule;

public:
	Simulator(Motor* m, PIDController* ctrl, Comparator *cmp,
		      SimulationResult *r, double target, double dt, double time);
	
	void scheduleLoad(double atTime, double load);
	void scheduleFriction(double atTime, double friction);
	
	void scheduleAllLoads(const std::vector<LoadEvent>& loads);
	void scheduleAllFriction(const std::vector<FrictionEvent>& frictions);
	void runSimulation();
};
	
