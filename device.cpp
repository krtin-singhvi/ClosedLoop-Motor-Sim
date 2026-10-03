#include "device.h"

Device::Device(double _Kp,
               double _Ki,
               double _Kd,
               double j,
               double b,
               double k,
               double tar,
               double DT,
               double ti)
{
    PIDController pidc(_Kp, _Ki, _Kd);

    Motor m(j, b, k);

    Comparator cmp;

    SimulationResult result;

    Simulator s(
        &m,
        &pidc,
        &cmp,
        &result,
        tar,
        DT,
        ti
    );
    s.runSimulation();
}
