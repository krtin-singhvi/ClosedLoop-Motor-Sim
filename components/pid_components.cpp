#include "pid_components.h"
#include <stdexcept>

Proportional::Proportional(double _Kp): Kp(_Kp), control(0){
    if(_Kp <= 0.0) throw std::invalid_argument("Proportional: Kp cannot be negative");
}
void Proportional::compute(double e_t, double dt)
{
    control = Kp * e_t;
}
double Proportional::getControl() const{
    return control;
}


Integral::Integral(double _Ki): Ki(_Ki), control(0){
    if(_Ki <= 0.0) throw std::invalid_argument("Integral: Ki cannot be negative");
}
void Integral::compute(double e_t, double dt)
{
    control += e_t * dt;
}
double Integral::getControl() const{
    return Ki*control;
}


Derivative::Derivative(double _Kd): Kd(_Kd), prev_e_t(0), control(0), firstCall(true){
    if(_Kd <= 0.0) throw std::invalid_argument("Proportional: Kd cannot be negative");
}
void Derivative::compute(double e_t, double dt)
{
    if(firstCall){
        control = 0;
        firstCall = false;
    }
    else{
        control = (e_t - prev_e_t) / dt;
    }
    prev_e_t = e_t;
}
double Derivative::getControl() const{
    return Kd*control;
}
