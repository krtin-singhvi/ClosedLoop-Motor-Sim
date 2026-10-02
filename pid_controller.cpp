#include "Controller.h"

Proportional::Proportional(double _Kp): Kp(_Kp), control(0){}

void Proportional::update(double Vin){ 
	this->control = Kp*Vin; 
}

double Proportional::getOutput() const{ 
	return this->control; 
}

Integrator::Integrator(double _Ki): Ki(_Ki), control(0){}

void Integrator::update(double e_t, double delta){
    this->control +=+ e_t*delta; //e-t is the error signal
}

double Integrator::getOutput() const{ 
	return Ki*control; 
}

Derivative::Derivative(double _Kd): Kd(_Kd), prev_e_t(0), control(0){}

void Derivative::update(double e_t, double delta) {
    control = (e_t - prev_e_t)/delta;
    prev_e_t = e_t; //might need to change how previous inputs are handled (either by class or by outside code)
}

double Derivative::getOutput() const { 
	return Kd*control; 
}

PIDController::PIDController(double Kp, double Ki, double Kd) : proportional(Kp), integrator(Ki), derivative(Kd) {}

double PIDController::compute(double error, double dt)
{
    proportional.update(error);
    integrator.update(error, dt);
    derivative.update(error, dt);
    return proportional.getOutput() + integrator.getOutput() + derivative.getOutput();
}
