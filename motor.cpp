#include <iostream>
#include "motor.h"

Motor::Motor(double _J, double _b, double _K): J(_J), b(_b), K(_K), w(0){}

void Motor::update(double _V, double delta){      
    V = _V;

    w = w + (K*V - b*w - T)*delta/J;
}
        
double Motor::getSpeed() const{
    return w;
}

void Motor::addLoad(double load){
    T += load;
}
void Motor::addFriction(double friction){
    b += friction;
}