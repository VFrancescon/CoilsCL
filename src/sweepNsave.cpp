#include <algorithm>
#include <opencv2/ximgproc.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <pylon/PylonIncludes.h>
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"

#define ONEMILLION 1000000

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
    
    
    
    
    
    
    /***************************
    WRITE YOUR CODE HERE
    ****************************/
    int counter = 1;
    int i = -25;

    mid.set3DField(-5, 0, 0);
    usleep(10*ONEMILLION);
    mid.set3DField(-10, 0, 0);
    usleep(10*ONEMILLION);
    mid.set3DField(-15, 0, 0);
    usleep(10*ONEMILLION);
    mid.set3DField(-20, 0, 0);
    usleep(10*ONEMILLION);
    mid.set3DField(-25, 0, 0);
    usleep(10*ONEMILLION);

    std::cout << "Initialisation complete. Press Enter to begin.";
    std::cin.get();

    while(camera.IsGrabbing()){
        outputFileName = "AlistairResults/" + expName + "step_" +  std::to_string(counter)  + 
        "field_" + std::to_string(i) + ".png";
        Pylon::String_t path = outputFileName.c_str();
        
        if(i > 25) {
            break;
            std::cout << "Breaking the loop\n";
        };

        float field = (float) i;
        std::cout << "\n\n--------------------NEW ITERATION--------------------\n\n";
        std::cout << "Setting field to " << field << "\n";
        mid.set3DField(field, 0, 0);
        i = i + 5;
        counter++;
        std::cout << "Field set. going to sleep for 10 seconds\n";
        
        // Sleep for some time
        usleep(15 * ONEMILLION);

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
