#include <HCoilMiddlewareLib/HCoilMiddlewareLib.hpp>
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#define WAITTIME 2
#include <chrono>
#include <condition_variable>

using namespace std::chrono_literals;


int main(int argc, char* argv[]){
    
    int maxAbs, step;

    if(argc == 3){
        bool PSU_ONLY = true;
        std::cout << "Before constructor\n";
        MiddlewareLayer mid(PSU_ONLY);

        maxAbs = std::stoi(argv[1]);
        step = std::stoi(argv[2]);

        std::cout << "Slowly stepping to -maxAbs\n";
        mid.set3DField(-maxAbs/4, 0, 0);
        std::cout << "One quarter" << -maxAbs/4  << "\n";
        usleep(WAITTIME*10e5);
        mid.set3DField(-maxAbs/2, 0, 0);
        std::cout << "Half" << -maxAbs/2 << "\n";
        usleep(WAITTIME*10e5);
        mid.set3DField(-maxAbs/4*3, 0, 0);
        std::cout << "Three quarter" << -maxAbs/4 * 3 << "\n";
        usleep(WAITTIME*10e5);


        std::cout << "Stepped to -maxAbs. Press enter to begin";
        std::cin.get();
        for(int i = -maxAbs; i < maxAbs+1; i = i + step){
            double field = (double) i;
            std::cout << "Setting field= " << field << "\n";
            mid.set3DField(field, 0, 0);
            std::cout << "Field set. going to sleep\n\n";
            usleep(WAITTIME*10e5);
        }
    } else{
        std::cout << "USAGE:\n";
        std::cout << "./sweep ARG1 ARG2\n";
        std::cout << "ARG1 provides the maximum absolute value to reach in the sweep\n";
        std::cout <<" ARG2 provides the step increase.\n";
        std::cout << "The programm will step to -ARG1 in 1/4 increments, then move from -ARG1 to +ARG1 in ARG2 increments.\n";
    }
    return 0;
}