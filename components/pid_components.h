#pragma once
class Proportional{
    private:
        const double Kp;
        double control;

    public:
        Proportional(double _Kp);
        void compute(double e_t, double dt);
        double getControl();
};

class Integral{
    private:
        const double Ki;
        double control;
    public:
        Integral(double _Ki);
        void compute(double e_t, double dt);
        double getControl();
        
};

class Derivative{
    private:
        const double Kd;
        double prev_e_t;
        double control;
        bool firstCall;

    public:
        Derivative(double _Kd);
        void compute(double e_t, double dt);
        double getControl();

};