#ifndef SIMULATIONRESULT_H
#define SIMULATIONRESULT_H

#include <vector>
#include <mutex>

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
    mutable std::mutex mtx;

public:
    void write(double time,double targetSpeed,double actualSpeed,double error,double controlInput);

    std::vector<SimulationData> getData() const;

    int size() const;
};

#endif