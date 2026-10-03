#include "controller.h"

bool compareSign(double a, double b){
    if(a > 0 && b > 0) return true;
    if(a < 0 && b < 0) return true;
    else return false;
}

Controller::Controller(double kp, double ki, double kd, double maxv, double minv): P(kp), I(ki), D(kd), clamp(maxv, minv){}

double Controller::compute(double error, double dt){

    P.compute(error, dt);
    D.compute(error, dt);

    double tentative = P.getControl() + I.getControl() + D.getControl();

    if(tentative != clamp.clamp(tentative) && compareSign(error, tentative)){
        I.compute(0, dt);
    }
    else{
        I.compute(error, dt);
    }

    return clamp.clamp(P.getControl() + I.getControl() + D.getControl());
}