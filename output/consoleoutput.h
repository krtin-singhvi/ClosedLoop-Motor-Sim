#ifndef CONSOLEOUTPUT_H
#define CONSOLEOUTPUT_H

#include "output.h"

class ConsoleOutput : public Output
{
public:
    void output(const SimulationResult& result) override;
};

#endif