#pragma once

class Motor{
    private:
        const double J; //inertia (kg.m^2)
        double b; //frictional term (N.m.s/rad)
        const double K; //motor constant (torque/voltage)
        double V; //current input voltage
        double w; //current angular speed
        double T; //external load torque (if any)
    
    public:
        Motor(double J, double b, double K);
        void updateSpeed(double V, double delta); //delta is the time step (dt)
        double getSpeed() const;
        void addLoad(double load);
        void addFriction(double friction);
};