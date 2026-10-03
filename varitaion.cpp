#include "variation.h"

Variation::Variation(double time, double loadChange, double frictionChange){
	this->time = time;
	this->loadChange = loadChange;
	this->frictionChange = frictionChange;
}

double Variation::getTime() const{
	return time;
}

double Variation::getLoadChange() const{
	return loadChange;
}

double Variation::getFrictionChange() const{
	return frictionChange;
}
