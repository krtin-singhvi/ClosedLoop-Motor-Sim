#include "components.h"
#include <iostream>
#include <stdexcept>

//Clamp
Clamp::Clamp(double _maxV, double _minV): maxV(_maxV), minV(_minV){
    if(_maxV < _minV) throw std::invalid_argument("Clamp: max voltage cannot be less than min voltage");
}

double Clamp::clamp(double input) const{
    if(input < maxV && input > minV) return input;

    return input >= maxV ? maxV : minV;
}

//Comparator
double Comparator::getError(double targetSpeed, double actualSpeed) const{
        return targetSpeed - actualSpeed;
}
