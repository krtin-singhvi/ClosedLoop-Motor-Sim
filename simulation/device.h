#pragma once

#include "pid_components.h"
#include "motor.h"
#include "simulator.h"

class Device {
private:
    Controller pidc;
    Motor motor;
    Comparator cmp;
    SimulationResult result;

public:
    Device(double _Kp,
           double _Ki,
           double _Kd,
           double maxv, 
           double minv,
           double j,
           double b,
           double k,
           double tar,
           double DT,
           double ti);
    const SimulationResult& getResult() const;
};
