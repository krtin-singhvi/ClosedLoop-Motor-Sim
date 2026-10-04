#ifndef SIMULATIONRESULT_H
#define SIMULATIONRESULT_H

#include <vector>

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
    std::vector<SimulationData> data;

public:
    void write(double time,double targetSpeed,double actualSpeed,double error,double controlInput);

    std::vector<SimulationData> getData() const;

    int size() const;
};

#endif