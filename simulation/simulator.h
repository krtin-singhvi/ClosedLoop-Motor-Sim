#pragma once
#include <vector>
#include "motor.h"
#include "components.h"
#include "controller.h"
#include "simulationresult.h"
#include "variation.h"

class Simulator{
private:
	Motor motor;
	Controller controller;
	Comparator comparator;
	SimulationResult result;
	double target, dt, totalTime;
	std::vector<Variation> variations;

public:
	Simulator(double _Kp,
    double _Ki,
    double _Kd,
    double maxv,
    double minv,
    double j,
    double b,
    double k,
    double target,
    double dt,
    double time);

	void addVariation();
	void runSimulation();
	SimulationResult returnResult() const;
};
	
