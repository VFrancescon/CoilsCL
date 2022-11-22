#include <DxkdpLib/DxkdpLib.hpp>
#include <LakeshoreF71Lib/LakeshoreF71Lib.hpp>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[]){

    std::string addrOne, addrTwo, addrTM;
    Teslameter::AXIS axisToRead;


    addrTM = "/dev/ttyUSB6";

    if(argc == 2){
        int valIn = std::stod(argv[1]);

        switch (valIn)
        {
        case 0:
            addrOne = "/dev/ttyUSB2";
            addrTwo = "/dev/ttyUSB3";
            axisToRead == Teslameter::AXIS::X;
            break;
        
        case 1:
            addrOne = "/dev/ttyUSB4";
            addrTwo = "/dev/ttyUSB5";
            axisToRead == Teslameter::AXIS::Z;
            break;

        default:
            return -1;
        }
    }


    DXKDP_PSU psu1(addrOne);
    DXKDP_PSU psu2(addrTwo);
    Teslameter tmeter(addrTM);


    std::cout << "Ready to go. Press enter";
    std::cin.get();

    //Step 1: 0 0
    std::cout << "Step 1: 0 0";
    psu1.setPolarity(0x00);
    psu2.setPolarity(0x00);
    std::cout << " read field " << tmeter.SingleAxisReading(axisToRead) << "\n";


    std::cout << "\n\n--------------------------------------\n\n";
    //Step 2: 0 1
    std::cout << "Step 1: 0 1";
    psu1.setPolarity(0x00);
    psu2.setPolarity(0x01);
    std::cout << " read field " << tmeter.SingleAxisReading(axisToRead) << "\n";

    std::cout << "\n\n--------------------------------------\n\n";
    //Step 2: 1 1
    std::cout << "Step 1: 1 1";
    psu1.setPolarity(0x01);
    psu2.setPolarity(0x01);
    std::cout << " read field " << tmeter.SingleAxisReading(axisToRead) << "\n";

    std::cout << "\n\n--------------------------------------\n\n";
    //Step 2: 1 0
    std::cout << "Step 1: 1 0";
    psu1.setPolarity(0x01);
    psu2.setPolarity(0x00);
    std::cout << " read field " << tmeter.SingleAxisReading(axisToRead) << "\n";


    return 0;
}