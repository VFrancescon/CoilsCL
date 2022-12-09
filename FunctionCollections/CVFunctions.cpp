#include <CVFunctions.hpp>
#include <cameraGeneric.hpp>


Mat cvFunctions::IntroducerMask(Mat src){
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
    this->intr_mask = src_GRAY;
    return src_GRAY;
}

std::vector<double> cvFunctions::computeAngles(std::vector<Point> Joints){
    std::vector<double> angles;
    std::vector<Point> vects;

    Joints.insert(Joints.begin(), Point(Joints[0].x, 0));
    for(int i = 1; i < Joints.size(); i++){
        vects.push_back(Point{Joints[i].x - Joints[i-1].x, Joints[i].y - Joints[i-1].y}  );
    }
    for(int i = 0; i < vects.size()-1; i++){
        double dproduct = vects[i].dot(vects[i+1]);
        double nproduct = norm(vects[i]) * norm(vects[i+1]);
        double th = acos(dproduct/nproduct);
        angles.push_back(th * 180 / M_PI);
    }
    std::reverse(angles.begin(), angles.end());
    return angles;
}

std::vector<Point> cvFunctions::computeIdealPoints(Point p0, std::vector<double> desiredAngles_){
    std::vector<Point> ideal;

    ideal.push_back(p0);
    for(int i = 1; i < desiredAngles_.size(); i++){
        double angle = 0;
        for( int k = 0; k < i; k++) angle += desiredAngles_[k];
        int xdiff = (link_length) * sin(angle * M_PI / 180);
        int ydiff = (link_length) * cos(angle * M_PI / 180);
        Point pn = Point{ (int) (ideal[i-1].x + xdiff), (int) ( ideal[i-1].y + ydiff )}; 
        ideal.push_back(pn);
    }

    return ideal;
}

std::vector<Point> cvFunctions::findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours){
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
    
    std::vector<Point> Joints;
    // int jointCount = (int) cntLine.size() / link_lenght;
    int jointNumber = 6;
    int link_l = cntLine.size() / jointNumber;

    for(int i = 0; i < jointNumber; i++){
        Joints.push_back(cntLine[link_l*i]);
    }

    // if(jointCount){
    //     for(int i = 0; i < jointCount; i++){
    //         Joints.push_back(cntLine[link_lenght*(i)]);
    //     }
    // }

    
    

    std::reverse(Joints.begin(), Joints.end());

    return Joints;
}

void cvFunctions::PylonSetup(){
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

}

void cvFunctions::PreProcessImage(Mat src, Mat dst){
    Mat src_grey, src_th;
    cvtColor(src, src_grey, COLOR_BGR2GRAY);
    blur(src_grey, src_grey, Size(5,5));
    threshold(src_grey, src_th, threshold_low, threshold_high, THRESH_BINARY_INV);
    src_th.copyTo(dst, this->intr_mask);
}