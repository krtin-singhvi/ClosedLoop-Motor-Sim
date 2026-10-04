#ifndef CSVOUTPUT_H
#define CSVOUTPUT_H

#include "output.h"
#include <string>
#include <fstream>

class CSVOutput : public Output
{
private:
    std::string filename;
    std::ofstream file;

public:

    CSVOutput(const std::string& filename);

    void output(const SimulationResult& result) override;

    void start();

    void outputRow(const SimulationData& row);

    void finish();
};

#endif
