#pragma once

class PIDController {
private:
    // Proportional
    const double Kp;
    double proportionalControl;

    // Integral
    const double Ki;
    double integralControl;

    // Derivative
    const double Kd;
    double derivativeControl;
    double prev_e_t;

public:
    PIDController(double Kp, double Ki, double Kd);

    // P component
    void updateProportional(double e_t);

    // I component
    void updateIntegral(double e_t, double delta);

    // D component
    void updateDerivative(double e_t, double delta);

    // Overall PID output
    double update(double error, double dt);
};
