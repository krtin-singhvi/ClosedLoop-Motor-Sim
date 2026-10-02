#include "PIDController.h"

PIDController::PIDController(double Kp, double Ki, double Kd) : proportional(Kp), integrator(Ki), derivative(Kd) {}

double PIDController::compute(double error,double dt)
{
    proportional.update(error);
    integrator.update(error,dt);
    derivative.update(error,dt);
    return proportional.getOutput()+integrator.getOutput()+derivative.getOutput();
}
