#include <iostream>
#include "motor.h"
#include "pid_components.h"

int main() {
    Motor motor(10, 5, 2);

    Proportional P(5.0);
    Integrator I(1.0);
    Derivative D(0.1);

    double target = 10.0;
    double dt = 0.001;

    for (int i = 0; i < 100000; i++) {

        // Measure current speed
        double speed = motor.getSpeed();

        // Calculate error
        double error = target - speed;

        // Update PID components
        P.update(error);
        I.update(error, dt);
        D.update(error, dt);

        // Total control voltage
        double voltage =
            P.getOutput() +
            I.getOutput() +
            D.getOutput();

        // Apply voltage to motor
        motor.update(voltage, 0, dt);

        // Print every 100 iterations
        if (i % 100 == 0) {
            std::cout
                << "Time: " << i * dt
                << "  Speed: " << speed
                << "  Error: " << error
                << "  Voltage: " << voltage
                << std::endl;
        }
    }
}