#pragma once

class Clamp{
    private:
        const double maxV;
        const double minV;
    public:
        Clamp(double _maxV, double _minV);
        double clamp(double input) const;
};

class Comparator{
public:
        double getError(double targetSpeed, double actualSpeed) const;
};