#include "device.h"

Device::Device(double _Kp,
               double _Ki,
               double _Kd,
               double maxv,
               double minv,
               double j,
               double b,
               double k,
               double target,
               double dt,
               double time)
    : pidc(_Kp, _Ki, _Kd, maxv, minv), motor(j, b, k)
{
    Simulator s(
        &motor,
        &pidc,
        &cmp,
        &result,
        target,
        dt,
        time
    );
    s.runSimulation();
}

const SimulationResult& Device::getResult() const {
    return result;
}
