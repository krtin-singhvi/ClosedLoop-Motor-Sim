#include "csvoutput.h"
#include <fstream>
#include <iostream>

using namespace std;

CSVOutput::CSVOutput(const string& filename)
    : filename(filename)
{
}

void CSVOutput::output(const SimulationResult& result)
{
    ofstream file(filename);

    if (!file.is_open())
    {
        throw "Error: Could not open file";
    }
    // this can be because of permission problem or invalid file name or 
    // directory doesn't exist

    file << "Time,TargetSpeed,ActualSpeed,Error,ControlInput\n";
    // this is done as that is written in " " goes into the file which is created 

    const vector<SimulationData>& data = result.getData();

    for (const SimulationData& row : data)
    {
        file << row.time << ","
             << row.targetSpeed << ","
             << row.actualSpeed << ","
             << row.error << ","
             << row.controlInput << "\n";
    }

    file.close();

    cout << "CSV file saved: "
         << filename << endl;
}