#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"
#include <cameraGeneric.hpp>
#include <ctime>

#define ONEMILLION 1000000
#define WAITTIME 5
using namespace cv;


int main(int argc, char * argv[])
{
    if(argc  > 1) {
        time_t curr_time;
        tm * curr_tm;
        time(&curr_time);
        curr_tm = localtime(&curr_time);
        char date_string[100];
        strftime(date_string, 50, "%d_%m_%y_", curr_tm);
        std::string date(date_string);
            
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
        Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(exposureTime);
        Size frameSize= Size((int)width.GetValue(), (int)height.GetValue());
        int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
        width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
        height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
        Pylon::CPixelTypeMapper pixelTypeMapper( &pixelFormat);
        Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
        Pylon::CGrabResultPtr ptrGrabResult;
        
        
        
        
        std::string outputFileName, expName;
        expName = argv[1];
        


        /***************************
        CAMERA SETUP HERE, IGNORE
        ****************************/
        std::cout << "Press enter to begin PSU Operation.";
        std::cin.get();
        usleep(5e6);
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* preImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        outputFileName = "JoshDavyResults/" + date + expName + "refPicture.png";
        Pylon::String_t path = outputFileName.c_str();
        pylonImage.Save(png, path);
        /***************************
        WRITE YOUR CODE HERE
        ****************************/
        int counter = 1;
        float i = 0.f;
        std::cout << "Initialising. setting field to -20\n";
        mid.set3DField(i, 0, 0);
        std::cout << "Initialisation complete. Press Enter to begin.";
        std::cin.get();

        while(camera.IsGrabbing()){
        while(true){
            outputFileName = "JoshDavyResults/" + date + expName + "_" + std::to_string(counter) 
            + "_" + std::to_string((int) i) + ".png";
            path = outputFileName.c_str();
            
            if(i > 20.f) {
                break;
                std::cout << "Breaking the loop\n";
            };

            float field =  i;
            std::cout << "\n\n--------------------NEW ITERATION--------------------\n\n";
            std::cout << "Setting field to " << field << "\n";
            mid.set3DField(field, 0, 0);
            i = i + 5.f;
            counter++;
            std::cout << "Field set. going to sleep for " << WAITTIME << " seconds\n";
            
            // Sleep for some time
            usleep(20e6);

            // Take a snapshot from the camera
            
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
            formatConverter.Convert(pylonImage, ptrGrabResult);
            pylonImage.Save(png, path);
            
            std::cout << "Picture saved. Moving onto next iteration\n";
            
        }

        return 0;
        // Pylon::PylonTerminate();
        }

        std::cout << "No arguments were given. Try again giving one argument for the experiment name\n";
        return 0;
    }
}
