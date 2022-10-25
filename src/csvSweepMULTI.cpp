#include <algorithm>
#include <opencv2/ximgproc.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <pylon/PylonIncludes.h>
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"

#define ONEMILLION 1000000
#define WAITTIME 10
using namespace cv;

std::vector<std::pair<double,double>> readCSV(std::string filename);


std::vector<std::pair<double,double>> readCSV(std::string filename){
    std::vector<std::pair<double,double>> readValues;

    std::ifstream file(filename, std::ios::in);
    std::string line, word;
    
    getline(file,line);
    getline(file,line);

    while(std::getline(file, line)){
        int counter = 0;
        std::stringstream sstr(line);
        // std::cout << line << "\n";
        std::pair<double,double> value;
        while(std::getline(sstr, word, ',')){

            if(counter == 1) {
                // std::cout << "value read: " << std::stod(word) << "\n";
                value.first = std::stod(word);
            }
            if(counter == 2) {
                // std::cout << "value read: " << std::stod(word) << "\n";
                value.second = std::stod(word);
            }
            counter++;
        }
        readValues.push_back(value);

    }

    return readValues;



}


int main(int argc, char* argv[]){
        
    /***************************
    INITIALISING THE MIDDLEWARE HERE, DO NOT TOUCH
    ****************************/
    bool psu_only = true;
    MiddlewareLayer mid(psu_only);


    /***************************
    CAMERA SETUP HERE, IGNORE
    ****************************/
    auto png = Pylon::EImageFileFormat::ImageFileFormat_Png;
    Pylon::PylonInitialize();
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylonImage;
    Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice() );
    camera.Open();
    Pylon::CIntegerParameter width     ( camera.GetNodeMap(), "Width");
    Pylon::CIntegerParameter height    ( camera.GetNodeMap(), "Height");
    Pylon::CEnumParameter pixelFormat  ( camera.GetNodeMap(), "PixelFormat");
    Size frameSize= Size((int)width.GetValue(), (int)height.GetValue());
    width.TrySetValue(640*3, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(480*3, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CPixelTypeMapper pixelTypeMapper( &pixelFormat);
    Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t* preImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    std::string outputFileName, expName;
    expName = argv[1];
    
    /***************************
    CAMERA SETUP HERE, IGNORE
    ****************************/
    
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    outputFileName = "AlistairResults/" + expName + "refPicture.png";
    Pylon::String_t path = outputFileName.c_str();
    pylonImage.Save(png, path);


    /***************************
    WRITE YOUR CODE HERE
    ****************************/
    int counter = 1;
    int i = 0;


    //reading csv here
    std::string inputPath1 = "../Angles1.csv";
    std::vector<std::pair<double,double>> readValues1 = readCSV(inputPath1);

    std::string inputPath2 = "../Angles2.csv";
    std::vector<std::pair<double,double>> readValues2 = readCSV(inputPath2);

    std::string inputPath3 = "../Angles3.csv";
    std::vector<std::pair<double,double>> readValues3 = readCSV(inputPath3);

    std::cout << "Initialisation complete. Press Enter to begin.";
    std::cin.get();

    int size1, size2, size3;
    size1 = readValues1.size();
    size2 = readValues2.size();
    size3 = readValues3.size();

    while(camera.IsGrabbing()){
        outputFileName = "AlistairResults/" + expName + "step_" +  std::to_string(counter)  + ".png";
        path = outputFileName.c_str();
        
    
        counter++;
        std::cout << "First dataset\n";
        for(auto k: readValues1){
            // Take a snapshot from the camera
            mid.set3DField(k.first, 0, k.second);
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
            formatConverter.Convert(pylonImage, ptrGrabResult);
            pylonImage.Save(png, path);
            std::cout << "Picture saved. Moving onto next iteration\n";
            usleep(WAITTIME * ONEMILLION);
        }

        std::cout << "Press enter to move onto second dataset.";
        std::cin.get();

        for(auto k: readValues2){
            mid.set3DField(k.first, 0, k.second);
            // Take a snapshot from the camera
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
            formatConverter.Convert(pylonImage, ptrGrabResult);
            pylonImage.Save(png, path);
            std::cout << "Picture saved. Moving onto next iteration\n";
            usleep(WAITTIME * ONEMILLION);
        }

        std::cout << "Press enter to move onto third dataset.";
        std::cin.get();

        for(auto k: readValues3){
            mid.set3DField(k.first, 0, k.second);
            // Take a snapshot from the camera
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
            formatConverter.Convert(pylonImage, ptrGrabResult);
            pylonImage.Save(png, path);
            std::cout << "Picture saved. Moving onto next iteration\n";
            usleep(WAITTIME * ONEMILLION);
        }

        if(counter > (size1 + size2 + size3 ) ) {
            mid.set3DField(0, 0, 0);
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
            formatConverter.Convert(pylonImage, ptrGrabResult);
            pylonImage.Save(png, path);
            break;
        }
    
    }
    mid.~MiddlewareLayer();
    Pylon::PylonTerminate();

    return 0;
}