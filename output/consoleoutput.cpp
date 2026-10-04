#include "consoleoutput.h"

#include <iostream>
#include <iomanip>

void ConsoleOutput::start()
{
    std::cout << "\n";
    std::cout << "Simulation Results\n";
    std::cout << "-------------------------------------------------------------\n";

    std::cout << std::setw(10) << "Time"
         << std::setw(15) << "Target"
         << std::setw(15) << "Speed"
         << std::setw(15) << "Error"
         << std::setw(15) << "Control"
         << "\n";

    std::cout << "-------------------------------------------------------------\n";
}

void ConsoleOutput::outputRow(const SimulationData& row)
{
    std::cout << std::setw(10) << row.time
         << std::setw(15) << row.targetSpeed
         << std::setw(15) << row.actualSpeed
         << std::setw(15) << row.error
         << std::setw(15) << row.controlInput
         << "\n";
}

void ConsoleOutput::finish()
{
    std::cout << "-------------------------------------------------------------\n";
}

void ConsoleOutput::output(const SimulationResult& result)
{
    start();

    const std::vector<SimulationData>& data =
        result.getData();

    for (const SimulationData& row : data)
    {
        outputRow(row);
    }

    finish();

    std::cout << "Total samples: "
         << result.size()
         << "\n";
}
