#ifndef CSVOUTPUT_H
#define CSVOUTPUT_H

#include "output.h"
#include <string>
#include <fstream>

using namespace std;

class CSVOutput : public Output
{
private:
    string filename;
    ofstream file;

public:

    CSVOutput(const string& filename);

    void output(const SimulationResult& result) override;

    void start();

    void outputRow(const SimulationData& row);

    void finish();
};

#endif
