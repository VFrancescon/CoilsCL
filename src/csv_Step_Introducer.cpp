#include <HCoilMiddlewareLib/HCoilMiddlewareLib.hpp>
#include <sstream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cameraGeneric.hpp>
#define WAITTIME 100e3

std::string home_path = "home/vittorio/";

std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours);

Mat IntroducerMask(Mat src)
{
    Mat src_GRAY, element;
    // create a greyscale copy of the image
    //  flip(src, src, 1);

    cvtColor(src, src_GRAY, COLOR_BGR2GRAY);

    // apply blur and threshold so that only the tentacle is visible
    blur(src_GRAY, src_GRAY, Size(5, 5));
    threshold(src_GRAY, src_GRAY, threshold_low, threshold_high, THRESH_BINARY_INV);

    element = getStructuringElement(MORPH_DILATE, Size(3, 3));
    dilate(src_GRAY, src_GRAY, element);

    bitwise_not(src_GRAY, src_GRAY);

    return src_GRAY;
}

int main(int argc, char *argv[])
{
    std::vector<float> bx, by, bz;
    std::string filename = "/home/vittorio/coil_manipulator/Uopt2_Nav2_OPT2RE.csv";
    if (argc == 2)
    {
        filename = argv[1];
    }

    std::ifstream file(filename, std::ios::in);
    std::string line, word;

    getline(file, line);

    while (std::getline(file, line))
    {
        int counter = 0;
        std::stringstream sstr(line);
        // std::cout << line << "\n";
        while (std::getline(sstr, word, ','))
        {

            if (counter == 6)
            {
                bx.push_back(std::stof(word) * 1);
                bx.push_back(std::stof(word) * 1);
                bx.push_back(std::stof(word) * 1);
            }
            if (counter == 7)
            {
                by.push_back(std::stof(word) * 1);
                by.push_back(std::stof(word) * 1);
                by.push_back(std::stof(word) * 1);
            }

            if (counter == 8)
            {
                bz.push_back(std::stof(word) * 1);
                bz.push_back(std::stof(word) * 1);
                bz.push_back(std::stof(word) * 1);
            }
            counter++;
        }
    }

    Mat pre_img, post_img, intr_mask;
    /*pylon video input here
    -----------------------------------------------------------*/
    Pylon::PylonInitialize();
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylonImage;
    Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    camera.Open();
    Pylon::CIntegerParameter width(camera.GetNodeMap(), "Width");
    Pylon::CIntegerParameter height(camera.GetNodeMap(), "Height");
    Pylon::CEnumParameter pixelFormat(camera.GetNodeMap(), "PixelFormat");
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(20000.0);
    Size frameSize = Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(1920, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(1216, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t *preImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
    /*-----------------------------------------------------------
    pylon video input here*/

    int rows = pre_img.rows / 2;
    int cols = pre_img.cols / 2;

    std::cout << rows * 1.5 << " " << cols * 1.5 << "\n";
    rows = rows * 1.5;
    cols = cols * 1.5;

    resize(pre_img, pre_img, Size(rows, cols), INTER_LINEAR);
    intr_mask = IntroducerMask(pre_img);

    // Point destination;
    // Point goal1(450,480);
    // Point goal2(250,650);

    // int bileinsertion = 45, straighinsertions = 50;
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

    VideoWriter video_out("output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
                          Size(rows, cols));
    int OP_MODE=1;
    MiddlewareLayer mid(OP_MODE);
    std::cout << "Everything initialised properly. Press enter to begin.";
    std::cin.get();

    int lastStep = bx.size();
    int step_count = 0;
    // 1. bile duct
    // Apply field
    //  mid.set3DField(-25, 0, -5);
    while (true)
    {
        // step 50 times (to the limit)
        mid.stepIntroducer();
        mid.set3DField(bx[step_count], by[step_count], bz[step_count]);
        step_count++;
        usleep(1e6);

        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

        if (post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rows, cols), INTER_LINEAR);
        

        // Mat post_img_grey, post_img_th, post_img_masked;

        // cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        // blur(post_img_grey, post_img_grey, Size(5,5));
        // threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        // post_img_th.copyTo(post_img_masked);

        // std::vector<Point> Joints;
        // std::vector<std::vector<Point>> contours;
        // Joints = findJoints(post_img_masked, contours);
        // int JointsObserved = Joints.size();
        // drawContours(post_img, contours, -1, Scalar(255,0,0), FILLED, LINE_8);

        // for(auto i: Joints){
        //     circle(post_img, i, 4, Scalar(255,0,0), FILLED);
        // }

        // destination = goal1;

        // line(post_img, Joints.front(), goal1, Scalar(255,255,0), 1);

        if(step_count == lastStep){
            video_out.write(post_img);
            break;
        }

        usleep(WAITTIME);
        imshow("Processed", post_img);
        video_out.write(post_img);
        char c = (char)waitKey(1);
        if (c == 27)
            break;
    }
    std::cout << "Reversing\n";
    std::vector<float> bxINV = bx, byINV = by, bzINV = bz;
    std::reverse(bxINV.begin(), bxINV.end());
    std::reverse(byINV.begin(), byINV.end());
    std::reverse(bzINV.begin(), bzINV.end());

        while (true)
    {
        // step 50 times (to the limit)
        mid.retractIntroducer();
        mid.set3DField(bxINV[step_count], byINV[step_count], bzINV[step_count]);
        step_count--;
        usleep(1e6);

        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

        if (post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rows, cols), INTER_LINEAR);
        

        // Mat post_img_grey, post_img_th, post_img_masked;

        // cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        // blur(post_img_grey, post_img_grey, Size(5,5));
        // threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        // post_img_th.copyTo(post_img_masked);

        // std::vector<Point> Joints;
        // std::vector<std::vector<Point>> contours;
        // Joints = findJoints(post_img_masked, contours);
        // int JointsObserved = Joints.size();
        // drawContours(post_img, contours, -1, Scalar(255,0,0), FILLED, LINE_8);

        // for(auto i: Joints){
        //     circle(post_img, i, 4, Scalar(255,0,0), FILLED);
        // }

        // destination = goal1;

        // line(post_img, Joints.front(), goal1, Scalar(255,255,0), 1);

        if(step_count == 0){
            video_out.write(post_img);
            break;
        }

        usleep(WAITTIME);
        imshow("Processed", post_img);
        video_out.write(post_img);
        char c = (char)waitKey(1);
        if (c == 27)
            break;
    }

    // Pylon::PylonTerminate();
    cv::destroyAllWindows();
    video_out.release();
    return 0;
}