#include <iostream>
#include "motor.h"

Motor::Motor(double _J, double _b, double _K): J(_J), b(_b), K(_K), w(0){}

void Motor::update(double _V, double _T, double delta){      
    T = _T;
    V = _V;

    w = w + (K*V - b*w - T)*delta/J;
}
        
double Motor::getSpeed() const{
    return w;
}


 // int main(){
//     Motor m(10, 5, 2);

//     for(int i = 0; i < 10000; i++){
//         m.update(5, 0, 0.001);
        
//         if(i%10 == 0){
//             std::cout << m.getSpeed() << std::endl;
//         }
//     }
// }
// test function for motor
