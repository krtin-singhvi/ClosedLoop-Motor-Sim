#pragma once
class Clamp{
    private:
        double maxV;
        double minV;
    public:
        Clamp(double _maxV, double _minV);
        double clamp(double input);
};

class Comparator{
public:
        double getError(double targetSpeed, double actualSpeed) const;
};