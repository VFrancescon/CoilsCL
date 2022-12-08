#include <DxkdpLib/DxkdpLib.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#define ONEMILLION 1000000

float xVoltage(float I){
    return 1.089f * I + 1.554f;
}


int main(int argc, char* argv[]){
    float currInput, voltInput;
    int sign;
    DXKDP_PSU psu("/dev/ttyUSB1", 0.1, 0.01);
    
    if(argc == 3){
        currInput = std::stof(argv[1]);
        sign = std::stoi(argv[2]);
        voltInput = xVoltage(currInput);

        if(sign == 0) psu.setPolarity(0x00);
        else psu.setPolarity(0x01);
        psu.PoCtrl(0x01);
        
        std::cout << "All ready to go.";
        std::cin.get();

        psu.WriteVI(voltInput, currInput);
        std::cout << "Press enter to quit";
        std::cin.get();

        psu.WriteVI(0,0);
        psu.PoCtrl(0x0);
        
        return 0;
    }



    std::cout << "ARG1: current I.\n ARG2: sign\n";
}
