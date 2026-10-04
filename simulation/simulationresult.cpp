#include "simulationresult.h"

void SimulationResult::write(double time,double targetSpeed,double actualSpeed,double error,double controlInput)
{
    SimulationData row;

    row.time = time;
    row.targetSpeed = targetSpeed;
    row.actualSpeed = actualSpeed;
    row.error = error;
    row.controlInput = controlInput;

    std::lock_guard<std::mutex> lock(mtx);
    data.push_back(row);
}

std::vector<SimulationData> SimulationResult::getData() const
{
    std::lock_guard<std::mutex> lock(mtx);
    return data;
}

int SimulationResult::size() const
{
    std::lock_guard<std::mutex> lock(mtx);
    return data.size();
}