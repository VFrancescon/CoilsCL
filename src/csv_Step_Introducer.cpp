#include <HCoilMiddlewareLib/HCoilMiddlewareLib.hpp>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
std::vector<std::string> row;
std::vector<float> bx, by, bz;

int main(int argc, char* argv[]){
    std::string filename = "/home/vittorio/coil_manipulator/Uopt2_Nav2_OPT2RE.csv";
    if(argc == 2) {
        filename = argv[1];
        }
    
    std::ifstream file(filename, std::ios::in);
    std::string line, word;
    
    getline(file,line);

    while(std::getline(file, line)){
        int counter = 0;
        std::stringstream sstr(line);
        // std::cout << line << "\n";
        while(std::getline(sstr, word, ',')){

            if(counter == 6) {
                bx.push_back(std::stof(word)* -3);
                bx.push_back(std::stof(word)* -3);
                bx.push_back(std::stof(word)* -3);
            }
            if(counter == 7) {
                by.push_back(std::stof(word) * -4);
                by.push_back(std::stof(word) * -4);
                by.push_back(std::stof(word) * -4);}
            
            if(counter == 8) {
                bz.push_back(std::stof(word) * -3);
                bz.push_back(std::stof(word) * -3);
                bz.push_back(std::stof(word) * -3);
            }
            counter++;
        }
    }
    std::cout << "Size of input: " << bx.size() << "\n";
    
    std::vector<float> bxINV = bx, byINV = by, bzINV = bz;
    std::reverse(bxINV.begin(), bxINV.end());
    std::reverse(byINV.begin(), byINV.end());
    std::reverse(bzINV.begin(), bzINV.end());


    // for(int i = 0; i < bx.size(); i++){
    //     std::cout << "Bx: " << bx[i] << " By: " << by[i] << " Bz: " << bz[i] <<"\n";    
    // }

    // MiddlewareLayer mid("/dev/ttyUSB2","/dev/ttyUSB3", "/dev/ttyUSB1", "/dev/ttyUSB4", "/dev/ttyUSB0");
    MiddlewareLayer mid;

    // std::cout << "Everything initialised properly.\nWe will write current to all PSUs starting at 0.01, 20 steps.\nPress enter to begin";
    std::cout << "Press enter to begin stepping through the csv";
    std::cin.get();
    
    // std::thread th1(&MiddlewareLayer::set3DVector, &mid, bx, by, bz);
    
    std::cout << "Inserting\n";
    mid.set3DVectorIN(bx, by, bz);
    std::cout << "Retracting\n";
    mid.set3DVectorOUT(bxINV, byINV, bzINV);
    

    //std::cout << "Field in X direction at the end of writing loop: " << mid.getXField();
}