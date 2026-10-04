#include <iostream>
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
    getVariations(s);
  
    s.runSimulation();
    
}
void Device::getVariations(Simulator& simulator)
{
    int numberOfVariations;

    cout << "Enter number of variations you want to add: ";
    cin >> numberOfVariations;

    for (int i = 0; i < numberOfVariations; i++)
    {
        double time;
        double loadChange;
        double frictionChange;

        cout << "\nVariation " << i + 1 << endl;

        cout << "Enter the time at which you want to introduce the variation (seconds): ";
        cin >> time;

        cout << "Enter load change (N.m): ";
        cin >> loadChange;

        cout << "Enter friction change (N.m.s/rad): ";
        cin >> frictionChange;

        simulator.addVariation(
            Variation(time, loadChange, frictionChange)
        );
    }
}
const SimulationResult& Device::getResult() const {
    return result;
}
