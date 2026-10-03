#include <iostream>
#include <exception>
#include "device.h"
#include "consoleoutput.h"

int main(){
    try{
    Device dev(10.0, 4.0, 0.01, 200.0, -200.0, 0.01, 0.1, 0.01, 20.0, 0.01, 25.0);
    ConsoleOutput out;
    out.output(dev.getResult());
    }
    catch(const std::exception& e){
        cout << e.what() << endl;
    }
}