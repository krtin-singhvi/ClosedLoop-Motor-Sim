#ifndef SIMULATIONRESULT_H
#define SIMULATIONRESULT_H

#include <vector>

using namespace std;

struct SimulationData
{
    double time;
    double targetSpeed;
    double actualSpeed;
    double error;
    double controlInput;
};

class SimulationResult
{
private:
    vector<SimulationData> data;

public:
    void write(double time,double targetSpeed,double actualSpeed,double error,double controlInput);

    const vector<SimulationData>& getData() const;

    int size() const;
};

#endif