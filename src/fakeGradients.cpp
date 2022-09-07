#include <DxkdpLib/DxkdpLib.hpp>

int main(int argc, char* argv[]){

    if(argc == 2){
        float inputField;
        inputField = std::stof(argv[1]);

        DXKDP_PSU PSU1("/dev/ttyUSB2");
        DXKDP_PSU PSU2("/dev/ttyUSB3");

        PSU1.WriteVI(60,0);
        PSU2.WriteVI(60,0);

        PSU1.setPolarity(0x01);
        PSU2.setPolarity(0x01);

        PSU1.PoCtrl(0x01);
        PSU2.PoCtrl(0x01);

        std::cout << "All set up. Press enter to begin";
        std::cin.get();

        PSU1.WriteCurrent(inputField*1.849);
        PSU2.WriteCurrent(inputField*1.849);

        std::cout << "Press enter to flip";
        std::cin.get();


        PSU1.setPolarity(0x00);
        PSU2.setPolarity(0x00);


        std::cout << "Press enter to quit";
        std::cin.get();

        PSU1.WriteVI(0,0);
        PSU2.WriteVI(0,0);
        PSU1.PoCtrl(0x00);
        PSU2.PoCtrl(0x00);
    }

    return 0;
}