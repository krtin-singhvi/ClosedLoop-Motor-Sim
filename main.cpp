#include <iostream>
#include <exception>
#include "simulator.h"
#include "consoleoutput.h"
#include "csvoutput.h"

using namespace std;

int main()
{
    try
    {
        Simulator sim(
            5.0,
            2,
            0.01,
            200.0,
            -200.0,
            0.01,
            0.05,
            0.1,
            20.0,
            0.01,
            15.0
        );

        sim.addVariation();
        sim.runSimulation();

        SimulationResult r = sim.returnResult();

        CSVOutput csv("simulation.csv");
        ConsoleOutput out;

        csv.output(r);
        out.output(r);


    }
    catch (const exception& e)
    {
        cout << e.what() << endl;
    }

    return 0;
}
