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
    // MiddlewareLayer mid(psu_only);


    /***************************
    CAMERA SETUP HERE, IGNORE
    ****************************/
    // auto png = Pylon::EImageFileFormat::ImageFileFormat_Png;
    // Pylon::PylonInitialize();
    // Pylon::CImageFormatConverter formatConverter;
    // formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    // Pylon::CPylonImage pylonImage;
    // Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice() );
    // camera.Open();
    // Pylon::CIntegerParameter width     ( camera.GetNodeMap(), "Width");
    // Pylon::CIntegerParameter height    ( camera.GetNodeMap(), "Height");
    // Pylon::CEnumParameter pixelFormat  ( camera.GetNodeMap(), "PixelFormat");
    // Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(20000.0);
    // Size frameSize= Size((int)width.GetValue(), (int)height.GetValue());
    // int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    // width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    // height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
    // Pylon::CPixelTypeMapper pixelTypeMapper( &pixelFormat);
    // Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());
    // camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    // Pylon::CGrabResultPtr ptrGrabResult;
    // camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    // const uint8_t* preImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    // formatConverter.Convert(pylonImage, ptrGrabResult);
    
    
    
    std::string outputFileName, expName;
    expName = argv[1];
    
    /***************************
    CAMERA SETUP HERE, IGNORE
    ****************************/
    // camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    // const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    // formatConverter.Convert(pylonImage, ptrGrabResult);
    // outputFileName = "FMP_TESTS/" + expName + "refPicture.png";
    // Pylon::String_t path = outputFileName.c_str();
    // pylonImage.Save(png, path);
    
    
    
    std::cout << "Press enter to begin";
    std::cin.get();
    /***************************
    WRITE YOUR CODE HERE
    ****************************/
    int counter = 1;
    float i = -20.f;

    // std::cout << "Initialising. setting field to -5\n";
    // mid.set3DField(-5, 0, 0);
    // usleep(6*ONEMILLION);
    // std::cout << "Initialising. setting field to -10\n";
    // mid.set3DField(-10, 0, 0);
    // usleep(6*ONEMILLION);
    // std::cout << "Initialising. setting field to -15\n";
    // mid.set3DField(-15, 0, 0);
    // usleep(6*2*ONEMILLION);
    std::cout << "Initialising. setting field to -20\n";
    // mid.set3DField(i, 0, 0);
    // usleep(6*ONEMILLION);
    DXKDP_PSU Psu0("/dev/ttyUSB0", 0.1, 0.01);
    DXKDP_PSU Psu1("/dev/ttyUSB1", 0.1, 0.01);
    DXKDP_PSU Psu2("/dev/ttyUSB2", 0.01, 0.01);
    // DXKDP_PSU Psu3("/dev/ttyUSB3", 0.1, 0.01);
    DXKDP_PSU Psu4("/dev/ttyUSB4", 0.01, 0.01);
    DXKDP_PSU Psu5("/dev/ttyUSB5", 0.01, 0.01);
    // DXKDP_PSU Psu1("/dev/ttyUSB1", 0.1, 0.01);
    
    std::cout << "Initialisation complete. Press Enter to begin.";
    std::cin.get();

    // while(camera.IsGrabbing()){
    // while(true){
    //     // outputFileName = "AlistairResults/" + expName + "step_" +  std::to_string(counter)  + 
    //     // "field_" + std::to_string(i) + ".png";
    //     // path = outputFileName.c_str();
        
    //     if(i > 20.f) {
    //         break;
    //         std::cout << "Breaking the loop\n";
    //     };

    //     float field =  i;
    //     std::cout << "\n\n--------------------NEW ITERATION--------------------\n\n";
    //     std::cout << "Setting field to " << field << "\n";
    //     mid.set3DField(field, 0, 0);
    //     i = i + 5.f;
    //     // counter++;
    //     std::cout << "Field set. going to sleep for " << WAITTIME << " seconds\n";
        
    //     // Sleep for some time
    //     usleep(5e6);

    //     // Take a snapshot from the camera
        
    //     // camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    //     // const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    //     // formatConverter.Convert(pylonImage, ptrGrabResult);
    //     // pylonImage.Save(png, path);
        
    //     // std::cout << "Picture saved. Moving onto next iteration\n";
        
    // }
    // DXKDP_PSU Psu("/dev/ttyUSB1");
    std::cout << "SETUP0\n";
    Psu0.WriteVI(60,0);
    Psu0.PoCtrl(0x01);
    std::cout << "SETUP0\n";

    std::cout << "SETUP1\n";
    Psu1.WriteVI(60,0);
    Psu1.PoCtrl(0x01);
    std::cout << "SETUP1\n";

    std::cout << "SETUP2\n";
    Psu2.WriteVI(60,0);
    Psu2.PoCtrl(0x01);
    std::cout << "SETUP2\n";

    std::cout << "SETUP4\n";
    Psu4.WriteVI(60,0);
    Psu4.PoCtrl(0x01);
    std::cout << "SETUP4\n";

    std::cout << "SETUP5\n";
    Psu5.WriteVI(60,0);
    Psu5.PoCtrl(0x01);
    std::cout << "SETUP5\n";

    
    for(int k = -20; k <= 20; k = k + 5){
        std::cout << "\n\n--------------------NEW ITERATION--------------------\n\n";
        std::cout << "Setting field to " << k << "\n";
        if( k < 0) {
            std::cout <<"Polarity0\n";
            Psu0.setPolarity(0x01);
            // usleep(5e6); 
            std::cout <<"Polarity1\n";
            Psu1.setPolarity(0x01);
            // usleep(5e6); 
            std::cout <<"Polarity2\n";
            Psu2.setPolarity(0x01); 
            // usleep(5e6); 
            Psu4.setPolarity(0x01); 
            Psu5.setPolarity(0x01); 
            
        }
        else {
            Psu0.setPolarity(0x00); 
            Psu1.setPolarity(0x00); 
            Psu2.setPolarity(0x00); 
            Psu4.setPolarity(0x00); 
            Psu5.setPolarity(0x00); 

        }
        Psu0.WriteCurrent( (float) abs(k));
        Psu1.WriteCurrent( (float) abs(k));
        Psu2.WriteCurrent( (float) abs(k));
        Psu4.WriteCurrent( (float) abs(k));
        Psu5.WriteCurrent( (float) abs(k));
        usleep(4e6);
    }
    Psu0.PoCtrl(0x00);
    Psu1.PoCtrl(0x00);
    Psu2.PoCtrl(0x00);
    Psu4.PoCtrl(0x00);
    Psu5.PoCtrl(0x00);
    // Psu1.PoCtrl(0x00);
    // mid.~MiddlewareLayer();
    return 0;
    // Pylon::PylonTerminate();
    }

    std::cout << "No arguments were given. Try again giving one argument for the experiment name\n";
    return 0;
}
