#include <QCoreApplication>

#include "mrtspserver.h"

using namespace cv;

void test(){
  RTSPServer mRtsp;
  mRtsp.Init("8888");
  VideoCapture cap("/home/cyx/programes/Videos/1.mp4");
//  std::cout <<"port: " << "8888" << std::endl;
  Mat img;
////  img.create(cv::Size(1920, 1080), CV_8UC3);
//  int i = 0;
//  while(true){
//      img.setTo(cv::Scalar(0,i,0));
//      mRtsp.sendFrameThread(img);
//      i++;
//      i%=255;
//    }
  int frame_count = cap.get(cv::CAP_PROP_FRAME_COUNT);
  int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  int count = 0;
  while(true){
      if(count >= frame_count){
          cap.release();
          cap = VideoCapture("/home/cyx/programes/Videos/1.mp4");
          count = 0;
        }
      cap.read(img);
      mRtsp.sendFrameThread(img);
      count++;
    }
}

int main(int argc, char *argv[])
{
  QCoreApplication a(argc, argv);

  test();

  return a.exec();
}
