#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc.hpp>
#include <pylon/PylonIncludes.h>
using namespace cv;

class cvFunctions{

public:

    Mat IntroducerMask(Mat src);
    std::vector<double> computeAngles(std::vector<Point> Joints);
    std::vector<Point> computeIdealPoints(Point p0, std::vector<double> desiredAngles_);
    std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours);
    void PylonSetup();
    void PreProcessImage(Mat src, Mat dst);

private: 
    Mat intr_mask;
};