#include <iostream>
#include "pid_components.h"


Proportional::Proportional(double _Kp): Kp(_Kp), control(0){}

void Proportional::update(double Vin){
    control = Kp*Vin;
}

double Proportional::getOutput() const{
    return control;
}




Integrator::Integrator(double _Ki): Ki(_Ki), control(0){}

void Integrator::update(double e_t, double delta){
    control = control + e_t*delta; //e-t is the error signal
}

double Integrator::getOutput() const{
    return Ki*control;
}



    
Derivative::Derivative(double _Kd): Kd(_Kd), prev_e_t(0), control(0){}

void Derivative::update(double e_t, double delta){

    control = (e_t - prev_e_t)/delta;
    prev_e_t = e_t; //might need to change how previous inputs are handled (either by class or by outside code)
}

double Derivative::getOutput() const{
    return Kd*control;
}
