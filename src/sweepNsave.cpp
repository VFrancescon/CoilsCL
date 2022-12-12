#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"
#include <cameraGeneric.hpp>

#define ONEMILLION 1000000
#define WAITTIME 5
using namespace cv;


int main(int argc, char * argv[])
{
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
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(20000.0);
    Size frameSize= Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
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
    usleep(WAITTIME*ONEMILLION);
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    outputFileName = "AlistairResults/" + expName + "refPicture.png";
    Pylon::String_t path = outputFileName.c_str();
    pylonImage.Save(png, path);
    
    
    
    std::cout << "Press enter to begin";
    std::cin.get();
    /***************************
    WRITE YOUR CODE HERE
    ****************************/
    int counter = 1;
    int i = -20;

    std::cout << "Initialising. setting field to -5\n";
    mid.set3DField(-5, 0, 0);
    usleep(5*ONEMILLION);
    std::cout << "Initialising. setting field to -10\n";
    mid.set3DField(-10, 0, 0);
    usleep(5*ONEMILLION);
    std::cout << "Initialising. setting field to -15\n";
    mid.set3DField(-15, 0, 0);
    usleep(5*ONEMILLION);
    std::cout << "Initialising. setting field to -20\n";
    mid.set3DField(-20, 0, 0);
    usleep(5*ONEMILLION);

    std::cout << "Initialisation complete. Press Enter to begin.";
    std::cin.get();

    while(camera.IsGrabbing()){
        outputFileName = "AlistairResults/" + expName + "step_" +  std::to_string(counter)  + 
        "field_" + std::to_string(i) + ".png";
        path = outputFileName.c_str();
        
        if(i > 20) {
            break;
            std::cout << "Breaking the loop\n";
        };

        float field = (float) i;
        std::cout << "\n\n--------------------NEW ITERATION--------------------\n\n";
        std::cout << "Setting field to " << field << "\n";
        mid.set3DField(field, 0, 0);
        i = i + 5;
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

    std::cout << "No arguments were given. Try again giving one argument for the experiment name\n";
    return 0;
}
