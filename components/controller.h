#pragma once
#include "components.h"
#include "pid_components.h"

class Controller{
    private:
        Proportional P;
        Integral I;
        Derivative D;
        Clamp clamp;  
        
    public:
        Controller(double kp, double ki, double kd, double maxv, double minv);
        double compute(double error, double dt);
};