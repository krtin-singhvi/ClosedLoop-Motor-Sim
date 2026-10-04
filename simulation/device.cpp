#include "device.h"

Device::Device(
    double _Kp,
    double _Ki,
    double _Kd,
    double maxv,
    double minv,
    double j,
    double b,
    double k,
    double target,
    double dt,
    double time
)
    : pidc(_Kp, _Ki, _Kd, maxv, minv),
      motor(j, b, k),
      cmp(),
      result(),
      simulator(
          &motor,
          &pidc,
          &cmp,
          &result,
          target,
          dt,
          time
      )
{
}

void Device::addVariation(
    const Variation& variation
)
{
    simulator.addVariation(variation);
}

void Device::runSimulation()
{
    simulator.runSimulation();
}

void Device::runMultiThreaded(
    ThreadSafeQueue<double>& controlQueue,
    ThreadSafeQueue<double>& speedQueue,
    ThreadSafeQueue<SimulationData>& consoleQueue,
    ThreadSafeQueue<SimulationData>& csvQueue
)
{
    simulator.runMultiThreaded(
        controlQueue,
        speedQueue,
        consoleQueue,
        csvQueue
    );
}

const SimulationResult& Device::getResult() const
{
    return result;
}
