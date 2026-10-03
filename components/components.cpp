#include "components.h"
#include <iostream>

//Clamp
Clamp::Clamp(double _maxV, double _minV): maxV(_maxV), minV(_minV){}

double Clamp::clamp(double input){
    if(input < maxV && input > minV) return input;

    return input >= maxV ? maxV : minV;
}

//Comparator
double Comparator::getError(double targetSpeed, double actualSpeed) const{
        return targetSpeed - actualSpeed;
}
