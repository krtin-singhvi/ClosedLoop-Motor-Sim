#pragma once
#include "pid_components.h"

class PIDController : {
	private:
    		Proportional proportional;
    		Integrator integrator;
    		Derivative derivative;

	public:
    		PIDController(double Kp,double Ki,double Kd);
    		double compute(double error,double dt);
};
