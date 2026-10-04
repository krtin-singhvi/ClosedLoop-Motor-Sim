#pragma once

#include "simulationresult.h"

class Output
{
public:
    virtual void output(const SimulationResult& result) = 0;

    virtual ~Output() = default;
};