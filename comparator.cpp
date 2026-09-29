#include "comparator.h"

double Comparator::getError(double targetSpeed, double actualSpeed) const{
	return targetSpeed - actualSpeed;
}
