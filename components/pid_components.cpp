#include "pid_components.h"

Proportional::Proportional(double _Kp): Kp(_Kp), control(0){}
void Proportional::compute(double e_t, double dt)
{
    control = Kp * e_t;
}
double Proportional::getControl(){
    return control;
}


Integral::Integral(double _Ki): Ki(_Ki), control(0){}
void Integral::compute(double e_t, double dt)
{
    control += e_t * dt;
}
double Integral::getControl(){
    return Ki*control;
}


Derivative::Derivative(double _Kd): Kd(_Kd), prev_e_t(0), control(0), firstCall(true){}
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
double Derivative::getControl(){
    return Kd*control;
}