#ifndef XSTITCHER_H
#define XSTITCHER_H

#include "xstitcher_global.h"
#include "opencv2/opencv.hpp"
#include <future>

using namespace std;
using namespace cv;

class RTSPServer;
class thread_pool;

struct four_corners_t
{
    Point2f left_top;
    Point2f left_bottom;
    Point2f right_top;
    Point2f right_bottom;
};


class XSTITCHERSHARED_EXPORT XStitcher
{

public:
    XStitcher();
    ~XStitcher();
    QString Init();
    bool release();
    void run();
private:
    bool isInit;
    thread_pool* pool = nullptr;

    RTSPServer* mrtsp = nullptr;
    std::vector<std::future<bool>> infs;
    std::vector<std::future<bool>> flgs;

    VideoCapture cap1, cap2, cap3, cap4;
    // store preprocessed imgs
    std::vector<Mat> processedImgs;
    std::vector<string> Cameras;

    // features related ---------------
    Mat homo1, homo2;
    Mat uparea, downarea, dst, up1920, down1920;
    four_corners_t corners1, corners2;
    // --------------------------------

    /*
    计算内参和畸变系数等
    */
    Mat cameraMatrix, distCoeffs;
    bool isCalibed = false;

    Mat YuvImg;

    // rtsp port
    std::string rtsp_port;

    void CalcCorners(const Mat& H, const Mat& src, four_corners_t& corners);

    void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst, four_corners_t& corners);

    Mat ExtractAndCalc(Mat& image01, Mat& image02);

    bool preprocessThread(int capIdx, Mat& output, double fx = 0.5, double fy = 0.5, bool cylindProjection = true);

    void Stitch_MultiThread();

    void cylindricalProjection(Mat& imgin, Mat& imgout);

    bool warpThread(Mat &img1, Mat& img2, Mat& area, Mat& dst, Mat &homo, four_corners_t& corners);

};

#endif // XSTITCHER_H
