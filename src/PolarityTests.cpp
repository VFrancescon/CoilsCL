#include <DxkdpLib/DxkdpLib.hpp>
#include <LakeshoreF71Lib/LakeshoreF71Lib.hpp>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[]){

    DXKDP_PSU psu1("/dev/ttyUSB1");

    psu1.WriteVI(10,5);
    usleep(5e5);
    psu1.PoCtrl(0x01);
    usleep(5e5);
    
    std::cout << "Press enter to begin polarity tests";
    std::cin.get();

    std::cout << "0x00";
    psu1.setPolarityGen2(0x00);
    std::cout << "Press enter to continue\n";
    std::cin.get();

    std::cout << "0x01";
    psu1.setPolarityGen2(0x01);
    std::cout << "Press enter to continue\n";
    std::cin.get();

    std::cout << "0x02";
    psu1.setPolarityGen2(0x02);
    std::cout << "Press enter to continue\n";
    std::cin.get();

    std::cout << "0x03";
    psu1.setPolarityGen2(0x03);
    std::cout << "Press enter to continue\n";
    std::cin.get();
    
    
    std::cout << "Quitting\n";
    
    return 0;
}