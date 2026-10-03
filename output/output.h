#ifndef OUTPUT_H
#define OUTPUT_H

#include "simulationresult.h"

class Output
{
public:
    virtual void output(const SimulationResult& result) = 0;

    virtual ~Output() = default;
};

#endif