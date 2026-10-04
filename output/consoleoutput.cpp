#include "consoleoutput.h"

#include <iostream>
#include <iomanip>

using namespace std;

void ConsoleOutput::start()
{
    cout << "\n";
    cout << "Simulation Results\n";
    cout << "-------------------------------------------------------------\n";

    cout << setw(10) << "Time"
         << setw(15) << "Target"
         << setw(15) << "Speed"
         << setw(15) << "Error"
         << setw(15) << "Control"
         << "\n";

    cout << "-------------------------------------------------------------\n";
}

void ConsoleOutput::outputRow(const SimulationData& row)
{
    cout << setw(10) << row.time
         << setw(15) << row.targetSpeed
         << setw(15) << row.actualSpeed
         << setw(15) << row.error
         << setw(15) << row.controlInput
         << "\n";
}

void ConsoleOutput::finish()
{
    cout << "-------------------------------------------------------------\n";
}

void ConsoleOutput::output(const SimulationResult& result)
{
    start();

    const vector<SimulationData>& data =
        result.getData();

    for (const SimulationData& row : data)
    {
        outputRow(row);
    }

    finish();

    cout << "Total samples: "
         << result.size()
         << "\n";
}
