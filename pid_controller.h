#pragma once

class Proportional {
	private:
    		const double Kp; //gain
    		double control; //output

	public:
    		Proportional(double Kp);
			void update(double e_t);
			double getOutput() const;
};


class Integrator {
	private:
			const double Ki; //gain
			double control; //output

	public:
    		Integrator(double Ki);
			void update(double e_t, double delta);
			double getOutput() const;
};


class Derivative {
	private:
    		const double Kd; //gain
    		double control; //output signal
    		double prev_e_t; //previous error signal to calculate derivative

	public:
    		Derivative(double Kd);
			void update(double e_t, double delta);
			double getOutput() const;
};


class PIDController {
	private:
    		Proportional proportional;
    		Integrator integrator;
    		Derivative derivative;

	public:
    		PIDController(double Kp, double Ki, double Kd);
			double compute(double error, double dt);
};
