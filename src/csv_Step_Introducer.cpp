#include <HCoilMiddlewareLib/HCoilMiddlewareLib.hpp>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
std::vector<std::string> row;
std::vector<float> bx, by, bz;

int main(int argc, char* argv[]){

    MiddlewareLayer mid;
    std::cout << "Everything initialised properly. Press enter to begin.";
    std::cin.get();
    //1. bile duct

    //Apply field
    mid.set3DField(-25, 0, -5);

    //step 50 times (to the limit)
    mid.stepIntroducer(50);

    //undo field and retract
    mid.set3DField(0,0,0);
    mid.retractIntroducer(50);

    //2. straight channnel
    
    //apply +bx
    mid.set3DField(25, 0, -5);
    mid.stepIntroducer(20);
    //apply -bx
    mid.set3DField(-25,0,-5);
    mid.stepIntroducer(20);
    //apply b=0
    mid.set3DField(0,0,0);
    mid.stepIntroducer(20);

    //3. Wrap up
    mid.retractIntroducer(60);

    return 0;

}