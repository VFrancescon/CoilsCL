#include <cameraGeneric.hpp>
#include <algorithm>
#include <opencv2/ximgproc.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <unistd.h>
using namespace cv;

int link_lenght_ = 50;

template<typename T>
double avgVect(std::vector<T> inputVec){
    double avg, sum = 0;

    for(auto i : inputVec){
        sum += i;
    }
    avg = sum / inputVec.size();
    return avg;
}


inline int meanError(std::vector<double> &desired, std::vector<double> &observed)
{
    double d_error, sum = 0;
    int error;
    // std::cout << "Inside meanError.\n";
    // std::cout << "Size of desired angles: " << desired.size() << "\nSize of observed angles: " << observed.size() << "\n";
    for (size_t i = 0; i < desired.size(); i++)
    {
        sum += desired[i] - observed[i];
    }
    
    d_error = sum / desired.size();
    if( std::isnan(d_error) ){
        std::cout << "Caught anomaly.";
        std::cout << "Error: " << d_error << "\n";
        std::cout << "sum: " << sum << "\n";
        std::cout << "desired.size(): " << desired.size() << "\n";
        return 15;
    }
    return (int)(d_error );
}

inline int pieceWiseError(std::vector<double> desired, std::vector<double> observed){
    double avg;
    int error;
    desired.pop_back();
    std::vector<double> diff(desired.size());
    if(desired.size() == observed.size()){
        for(int i = 0; i < desired.size(); i++){
            diff[i] = desired[i] - observed[i];
        }
        avg = avgVect(diff);
        return (int) avg;
    } else {
        std::cout << "No matching sizes\n";
        return 0;}
}

std::vector<Point> computeIdealPoints(Point p0, std::vector<double> desiredAngles_){
    std::vector<Point> ideal;

    ideal.push_back(p0);
    for(int i = 1; i < desiredAngles_.size(); i++){
        double angle = 0;
        for( int k = 0; k < i; k++) angle += desiredAngles_[k];
        int xdiff = (link_lenght_) * sin(angle * M_PI / 180);
        int ydiff = (link_lenght_) * cos(angle * M_PI / 180);
        Point pn = Point{ (int) (ideal[i-1].x + xdiff), (int) ( ideal[i-1].y + ydiff )}; 
        ideal.push_back(pn);
    }

    return ideal;
}


std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours)
{

    Mat contours_bin;
    // std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    // find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    // draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255, 255, 255), cv::FILLED, LINE_8, hierarchy);
    // empty matrix. Set up to 8-bit 1 channel data. Very important to set up properly.
    Mat skeleton = Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);

    // take the filled contour and thin it using Zhang Suen method. Only works with 8-bit 1 channel data.
    ximgproc::thinning(contours_bin, skeleton, 0);

    contours.clear();
    hierarchy.clear();
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);
    std::sort(cntLine.begin(), cntLine.end(), yWiseSort);
    // std::sort(cntLine.begin(), cntLine.end(), xWiseSort);
    // std::reverse(cntLine.begin(), cntLine.end());

    std::vector<Point> Joints;
    int jointCount = (int) std::ceil( (float) (cntLine.size() / (float) link_lenght_));
    // std::cout << "Size of centre-line " << cntLine.size() << "\n"
    // << "JointCount: " << jointCount << "\n";

    if (jointCount)
    {
        std::vector<Point>::iterator cntLineIterator = cntLine.begin();
        for(int i = 0; i < jointCount; i++){
            Joints.push_back(*cntLineIterator);
            std::advance(cntLineIterator, link_lenght_);
        }

    }
    std::reverse(Joints.begin(), Joints.end());
    // std::cout << "Number of joints " << Joints.size() << "\n";

    return Joints;
}

std::vector<double> computeAngles(std::vector<Point> Joints){
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

    return angles;

}

int main(int argc, char* argv[]){
    int th_low = 190;
    std::string img_path = "string";
    std::vector<double> DesiredAngles(6), ObservedAngles;
    //mag tray
    DesiredAngles[0] = 15;
    DesiredAngles[1] = -15;
    DesiredAngles[2] = 90;
    DesiredAngles[3] = -90;
    DesiredAngles[4] = 90;
    DesiredAngles[5] = 0;

    //10/20/30/45/30
    std::vector<double> RealistcAngles(6);
    //mag tray
    RealistcAngles[0] = 10;
    RealistcAngles[1] = 20;
    RealistcAngles[2] = 30;
    RealistcAngles[3] = 45;
    RealistcAngles[4] = 30;
    RealistcAngles[5] = 0;
    
    if(argc >= 2) {
        img_path = argv[1];
        std::cout << "Input path: " << img_path<< "\n";
        }

    if(argc == 3) {
        th_low = std::stoi(argv[2]);
        std::cout << "Th_low: " << th_low << "\n";
        }


    std::vector<std::string> FileNames;
    cv::String folderPath = img_path + "*.png";
    glob(folderPath, FileNames, false);

    for( auto k1: FileNames){
        Mat img  = imread(k1, IMREAD_COLOR);
        int rows,cols;
        rows = img.rows / 8 * 3;
        cols = img.cols / 8 * 3;



        resize(img, img, Size(cols,rows), INTER_LINEAR);
        Mat img_copy, threshold_img, img_post, cnts_bin, skeleton;

        //create a greyscale copy of the image
        cvtColor(img, img_copy, COLOR_BGR2GRAY);
        
        //apply blur and threshold so that only the tentacle is visible
        blur(img_copy, img_copy, Size(5,5));
        threshold(img_copy, threshold_img, th_low, 255, THRESH_BINARY_INV); 

        //set up vectors for findContours to output to
        std::vector<std::vector<Point> > contours;
        std::vector<Vec4i> hierarchy;
        std::vector<Point> Joints;
        
        Joints = findJoints(threshold_img, contours);
        int cntr = 0;
        for(auto i: Joints){
            circle(img, i, 4, Scalar(255,0,0), FILLED);
            // putText(img, std::to_string(cntr), i, FONT_HERSHEY_SIMPLEX, 2, Scalar(0,0,0));
            cntr++;
        }
        // circle(img, Joints[Joints.size()-1], 4, Scalar(255,255,0), FILLED);

        ObservedAngles = computeAngles(Joints);
        // std::cout << "Observed Angles\n";
        // for(auto i: ObservedAngles){
        //     std::cout << i << "\n";
        // }

        std::vector<Point> idealPoints = computeIdealPoints(Joints[0], DesiredAngles);
        for (int i = 0; i < idealPoints.size() - 1; i++)
        {
            line(img, idealPoints[i], idealPoints[i + 1], Scalar(0, 0, 255));
            circle(img, idealPoints[i], 2, Scalar(255, 0, 0));
            // idealPoints[i] *= -1;
        }

        std::vector<Point> RealistcPoints = computeIdealPoints(Joints[0], RealistcAngles);
        for (int i = 0; i < RealistcPoints.size() - 1; i++)
        {
            line(img, RealistcPoints[i], RealistcPoints[i + 1], Scalar(255, 0, 255));
            circle(img, RealistcPoints[i], 2, Scalar(255, 0, 255));
            // RealistcPoints[i] *= -1;

        }   

        // // int idealError = meanError(DesiredAngles, ObservedAngles);
        // int realisticError = meanError(RealistcAngles, ObservedAngles);


        int idealError = pieceWiseError(DesiredAngles, ObservedAngles);
        int realisticError = pieceWiseError(RealistcAngles, ObservedAngles);

        std::cout << "\n--------------------NEW ITERATION--------------------\n";
        std::cout << "Ideal error: " << idealError << "\n";
        std::cout << "Realistic error: " << realisticError << "\n";
        //image specific section

        imshow("Joints", img);
        char c= (char) waitKey(0);
    }

    return 0;
}