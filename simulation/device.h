#pragma once

#include "pid_components.h"
#include "motor.h"
#include "simulator.h"

class Device
{
private:

    Controller pidc;
    Motor motor;
    Comparator cmp;
    SimulationResult result;
    Simulator simulator;

public:

    Device(
        double _Kp,
        double _Ki,
        double _Kd,
        double maxv,
        double minv,
        double j,
        double b,
        double k,
        double tar,
        double DT,
        double ti
    );

    void addVariation(
        const Variation& variation
    );

    void runSimulation();

    void runMultiThreaded(
        ThreadSafeQueue<double>& controlQueue,
        ThreadSafeQueue<double>& speedQueue,
        ThreadSafeQueue<SimulationData>& consoleQueue,
        ThreadSafeQueue<SimulationData>& csvQueue
    );

    const SimulationResult& getResult() const;
};
