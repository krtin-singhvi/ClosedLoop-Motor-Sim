#ifndef CONSOLEOUTPUT_H
#define CONSOLEOUTPUT_H

#include "output.h"
#include "simulationresult.h"

class ConsoleOutput : public Output
{
public:

    void output(const SimulationResult& result) override;

    void start();

    void outputRow(const SimulationData& row);

    void finish();
};

#endif
