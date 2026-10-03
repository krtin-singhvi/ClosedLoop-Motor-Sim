#include "consoleoutput.h"
#include <iostream>
#include <iomanip>

using namespace std;

void ConsoleOutput::output(const SimulationResult& result)
{
    const vector<SimulationData>& data = result.getData();

    cout << "\n";

    cout << "Simulation Results\n";

    cout << "-------------------------------------------------------------\n";
    cout << setw(10) << "Time"<< setw(15) << "Target"<< setw(15) << "Speed"<< setw(15) << "Error"<< setw(15) << "Control" << "\n";

    cout << "-------------------------------------------------------------\n";

    for (const SimulationData& row : data)
    {
        cout << setw(10) << row.time<< setw(15) << row.targetSpeed<< setw(15) << row.actualSpeed<< setw(15) << row.error<< setw(15) << row.controlInput<< "\n";
    }

    cout << "-------------------------------------------------------------\n";

    cout << "Total samples: "<< result.size()<< "\n";
}
