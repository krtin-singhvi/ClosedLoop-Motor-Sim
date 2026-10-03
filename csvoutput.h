#ifndef CSVOUTPUT_H
#define CSVOUTPUT_H

#include "output.h"
#include <string>

using namespace std;

class CSVOutput : public Output
{
private:
    string filename;

public:
    CSVOutput(const string& filename);

    void output(const SimulationResult& result) override;
};

#endif