#include "simulationresult.h"

void SimulationResult::write(double time,double targetSpeed,double actualSpeed,double error,double controlInput)
{
    SimulationData row;

    row.time = time;
    row.targetSpeed = targetSpeed;
    row.actualSpeed = actualSpeed;
    row.error = error;
    row.controlInput = controlInput;

    data.push_back(row);
}

std::vector<SimulationData> SimulationResult::getData() const
{
    return data;
}

int SimulationResult::size() const
{
    return data.size();
}