#include "csvoutput.h"

#include <iostream>
#include <stdexcept>

using namespace std;

CSVOutput::CSVOutput(const string& filename)
    : filename(filename)
{
}

void CSVOutput::start()
{
    file.open(filename);

    if (!file.is_open())
    {
        throw runtime_error(
            "Error: Could not open file: " + filename
        );
    }

    file << "Time,TargetSpeed,ActualSpeed,Error,ControlInput\n";
}

void CSVOutput::outputRow(const SimulationData& row)
{
    file << row.time << ","
         << row.targetSpeed << ","
         << row.actualSpeed << ","
         << row.error << ","
         << row.controlInput << "\n";
}

void CSVOutput::finish()
{
    file.close();

    cout << "CSV file saved: "
         << filename
         << endl;
}

void CSVOutput::output(const SimulationResult& result)
{
    start();

    const vector<SimulationData>& data =
        result.getData();

    for (const SimulationData& row : data)
    {
        outputRow(row);
    }

    finish();
}
