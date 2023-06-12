#include <fstream>
#include <chrono>
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"
#include <cameraGeneric.hpp>
#include <algorithm>
#include <opencv2/ximgproc.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <unistd.h>

int main(int argc, char* argv[]){
    /***************************
    CAMERA SETUP HERE, IGNORE
    ****************************/

    bool psu_only_mode = true; 
    MiddlewareLayer mid(psu_only_mode);
    int counter = -12;
    std::cout << "Press enter to begin PSU Operation.";
    std::cin.get();



    while(true){

        //make image smaller 
        std::cout << "Counter at: " << counter << "\n";
        mid.set3DField(counter, 0, -counter);
        counter += 4;
        usleep(75e5);

        if(counter > 12) counter = -12;
        
    }

}