#include <algorithm>
#include <opencv2/ximgproc.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <pylon/PylonIncludes.h>
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"

#define ONEMILLION 1000000
#define WAITTIME 2
using namespace cv;

std::vector<std::pair<double,double>> readCSV(std::string filename);


std::vector<std::pair<double,double>> readCSV(std::string filename){
    std::cout << "Function called\n";
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
            readValues.push_back(value);
            counter++;
        }
    }

    return readValues;



}


int main(int argc, char* argv[]){
    if(argc  > 1) {
        
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
    std::string inputPath;
    if(argc > 3) inputPath = argv[2];
    else inputPath = "../Angles.csv";
    std::vector<std::pair<double,double>> readValues = readCSV(inputPath);

    std::cout << "Initialisation complete. Press Enter to begin.";
    std::cin.get();

    while(camera.IsGrabbing()){
        outputFileName = "AlistairResults/" + expName + "step_" +  std::to_string(counter)  + 
        "field_" + std::to_string(i) + ".png";
        path = outputFileName.c_str();
        
        std::cout << "\n\n--------------------NEW ITERATION--------------------\n\n";
        std::cout << "Setting field to " << readValues[i].first << "," << 0 << "," << readValues[i].second << "\n";
        mid.set3DField(readValues[i].first, 0, readValues[i].second);
        i++;
        counter++;
        std::cout << "Field set. going to sleep for " << WAITTIME << " seconds\n";
        
        // Sleep for some time
        usleep(WAITTIME * ONEMILLION);

        // Take a snapshot from the camera
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        pylonImage.Save(png, path);
        std::cout << "Picture saved. Moving onto next iteration\n";
        
    }
    mid.~MiddlewareLayer();
    Pylon::PylonTerminate();
    }

    std::cout << "No arguments were given. Try again giving one argument for the experiment name and one for the csv path\n";
    return 0;
}