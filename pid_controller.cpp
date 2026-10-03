#include "pid_Controller.h"

PIDController::PIDController(double _Kp, double _Ki, double _Kd) : Kp(_Kp), proportionalControl(0), Ki(_Ki), integralControl(0), Kd(_Kd), derivativeControl(0), prev_e_t(0) {}

// Proportional component
void PIDController::updateProportional(double e_t)
{
    proportionalControl = Kp * e_t;
}

// Integral component
void PIDController::updateIntegral(double e_t, double delta)
{
    integralControl += e_t * delta;
}

// Derivative component
void PIDController::updateDerivative(double e_t, double delta)
{
    derivativeControl = (e_t - prev_e_t) / delta;
    prev_e_t = e_t;
}

// Overall PID controller
double PIDController::compute(double error, double dt)
{
    updateProportional(error);
    updateIntegral(error, dt);
    updateDerivative(error, dt);
    return proportionalControl + Ki * integralControl + Kd * derivativeControl;
}
