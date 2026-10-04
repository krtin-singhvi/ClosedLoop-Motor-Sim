#include <iostream>
#include <exception>
#include <thread>

#include "device.h"
#include "consoleoutput.h"
#include "csvoutput.h"
#include "threadsafequeue.h"

using namespace std;

int main()
{
    try
    {
        Device dev(
            10.0,
            4.0,
            0.01,
            200.0,
            -200.0,
            0.01,
            0.1,
            0.01,
            20.0,
            0.01,
            25.0
        );

        int numberOfVariations;

        cout << "Enter number of variations: ";
        cin >> numberOfVariations;

        for (int i = 0; i < numberOfVariations; i++)
        {
            double time;
            double loadChange;
            double frictionChange;

            cout << "\nVariation " << i + 1 << endl;

            cout << "Enter time (seconds): ";
            cin >> time;

            cout << "Enter load change: ";
            cin >> loadChange;

            cout << "Enter friction change: ";
            cin >> frictionChange;

            dev.addVariation(
                Variation(
                    time,
                    loadChange,
                    frictionChange
                )
            );
        }

        ThreadSafeQueue<double> controlQueue;

        ThreadSafeQueue<double> speedQueue;

        ThreadSafeQueue<SimulationData>
            consoleQueue;

        ThreadSafeQueue<SimulationData>
            csvQueue;

        ConsoleOutput consoleOutput;

        CSVOutput csvOutput(
            "simulation.csv"
        );

        /*
         * CONSOLE THREAD
         */
        thread consoleThread([&]()
        {
            consoleOutput.start();

            SimulationData data;

            while (consoleQueue.pop(data))
            {
                consoleOutput.outputRow(data);
            }

            consoleOutput.finish();
        });


        /*
         * CSV THREAD
         */
        thread csvThread([&]()
        {
            csvOutput.start();

            SimulationData data;

            while (csvQueue.pop(data))
            {
                csvOutput.outputRow(data);
            }

            csvOutput.finish();
        });


        /*
         * START SIMULATION
         */
        dev.runMultithreaded(
            controlQueue,
            speedQueue,
            consoleQueue,
            csvQueue
        );


        /*
         * WAIT FOR OUTPUT THREADS
         */
        consoleThread.join();
        csvThread.join();

        cout << "\nSimulation finished successfully."
             << endl;

        cout << "Total samples: "
             << dev.getResult().size()
             << endl;
    }
    catch (const exception& e)
    {
        cout << e.what() << endl;
    }

    return 0;
}
