#include <HCoilMiddlewareLib/HCoilMiddlewareLib.hpp>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cameraGeneric.hpp>

int threshold_low = 20;
int threshold_high = 255;
std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours);

Mat IntroducerMask(Mat src){
    Mat src_GRAY, element;
    //create a greyscale copy of the image
    // flip(src, src, 1);

    cvtColor(src, src_GRAY, COLOR_BGR2GRAY);
    
    //apply blur and threshold so that only the tentacle is visible
    blur(src_GRAY, src_GRAY, Size(5,5));
    threshold(src_GRAY, src_GRAY, threshold_low, threshold_high, THRESH_BINARY_INV); 
    
    element = getStructuringElement(MORPH_DILATE, Size(3,3) );
    dilate(src_GRAY, src_GRAY, element);
    
 
    bitwise_not(src_GRAY, src_GRAY);

    return src_GRAY;

}

int main(int argc, char* argv[]){
    Mat pre_img, post_img, intr_mask;
    /*pylon video input here
    -----------------------------------------------------------*/
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
    width.TrySetValue(1920, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(1216, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CPixelTypeMapper pixelTypeMapper( &pixelFormat);
    Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t* preImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
    /*-----------------------------------------------------------
    pylon video input here*/
    
    int rcols, rrows;
    rcols = pre_img.rows;
    rrows = pre_img.cols;

    resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
    intr_mask = IntroducerMask(pre_img);

    AStar::Vec2i origin, destination, wordlsize;
    wordlsize.x = pre_img.rows;
    wordlsize.y = pre_img.cols;

    AStar::Generator generator;
    generator.setWorldSize(wordlsize);
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);
    Point goal1(60,60);


    /**
     * How to use the A-star pathfinding
     * 
     * starting_point = origin
     * destination = goal
     * AStar::CoordinateList path = generator.findPath(origin, destination);
     * std::vector<Point> cvPath = AStar::Vec2iToCvPointList(path);
     *      std::vector<Point> goalPath;
            for(auto i: path){
                goalPath.push_back(Point(i.x, i.y));
            }
            for(auto i: goalPath){
                circle(img, i, 2, Scalar(255,0,0), FILLED);
            }
     */


    MiddlewareLayer mid;
    std::cout << "Everything initialised properly. Press enter to begin.";
    std::cin.get();

    int step_count = 0;
    //1. bile duct
    //Apply field
    mid.set3DField(-25, 0, -5);
    while(true){

        if(step_count > 50) break;

        

        //step 50 times (to the limit)
        mid.stepIntroducer();
        step_count++;

    
    
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

        if(post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rrows, rcols), INTER_LINEAR);
        Mat post_img_grey, post_img_th, post_img_masked;

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5,5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point>> contours;
        Joints = findJoints(post_img_masked, contours);
        int JointsObserved = Joints.size();
        drawContours(post_img, contours, -1, Scalar(255,0,0), FILLED, LINE_8);

        for(auto i: Joints){
            circle(post_img, i, 4, Scalar(255,0,0), FILLED);        
        }
    
    }

    //undo field and retract
    mid.set3DField(0,0,0);
    mid.retractIntroducer(50);
    step_count = 0;

    //2. straight channnel
    
    //apply +bx
    mid.set3DField(25, 0, -5);
    while(true){
        if(step_count > 20) break;

        
        mid.stepIntroducer();
        step_count++;

        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

        if(post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rrows, rcols), INTER_LINEAR);
        Mat post_img_grey, post_img_th, post_img_masked;

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5,5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point>> contours;
        Joints = findJoints(post_img_masked, contours);
        int JointsObserved = Joints.size();
        drawContours(post_img, contours, -1, Scalar(255,0,0), FILLED, LINE_8);

        for(auto i: Joints){
            circle(post_img, i, 4, Scalar(255,0,0), FILLED);        
        }
    }
    //apply -bx
    mid.set3DField(-25,0,-5);
    while(true){
        if(step_count > 20) break;

        
        mid.stepIntroducer();
        step_count++;

        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

        if(post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rrows, rcols), INTER_LINEAR);
        Mat post_img_grey, post_img_th, post_img_masked;

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5,5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point>> contours;
        Joints = findJoints(post_img_masked, contours);
        int JointsObserved = Joints.size();
        drawContours(post_img, contours, -1, Scalar(255,0,0), FILLED, LINE_8);

        for(auto i: Joints){
            circle(post_img, i, 4, Scalar(255,0,0), FILLED);        
        }
    }
    //apply b=0
    mid.set3DField(0,0,0);
        while(true){
        if(step_count > 20) break;

        
        mid.stepIntroducer();
        step_count++;

        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

        if(post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rrows, rcols), INTER_LINEAR);
        Mat post_img_grey, post_img_th, post_img_masked;

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5,5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point>> contours;
        Joints = findJoints(post_img_masked, contours);
        int JointsObserved = Joints.size();
        drawContours(post_img, contours, -1, Scalar(255,0,0), FILLED, LINE_8);

        for(auto i: Joints){
            circle(post_img, i, 4, Scalar(255,0,0), FILLED);        
        }
    }
    
    //3. Wrap up
    mid.retractIntroducer(60);

    return 0;

}


std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours){
    

    Mat contours_bin;

    // std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    //find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    //draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255,255,255), cv::FILLED, LINE_8, hierarchy);
    //empty matrix. Set up to 8-bit 1 channel data. Very important to set up properly.
    Mat skeleton = Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);
    
    //take the filled contour and thin it using Zhang Suen method. Only works with 8-bit 1 channel data. 
    ximgproc::thinning(contours_bin, skeleton, 0);
    
    contours.clear();
    hierarchy.clear();
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);
    std::sort(cntLine.begin(), cntLine.end(), yWiseSort);
    // std::reverse(cntLine.begin(), cntLine.end());
    
    int link_lenght = 70;
    std::vector<Point> Joints;
    int jointCount = (int) cntLine.size() / link_lenght;
    
    
    if(jointCount){
        for(int i = 0; i < jointCount; i++){
            Joints.push_back(cntLine[link_lenght*(i)]);
        }
    }

    return Joints;
}