#include "xstitcher.h"
#include<chrono>
#include<QSettings>
#include<opencv2/xfeatures2d.hpp>
#include "mrtspserver.h"
#include "thread_pool.h"

#define pi 3.14

using namespace chrono;

XStitcher::XStitcher(){
    if(mrtsp == nullptr){
        mrtsp = new RTSPServer();
        cout << " create rtsp server ... done\n";
    }
    if(pool == nullptr){
        pool = new thread_pool();\
        cout << "create thread pool... done\n";
    }
    isInit = false;
}

XStitcher::~XStitcher(){
    if(mrtsp != nullptr){
        delete mrtsp;
        mrtsp = nullptr;
    }
    if(pool != nullptr){
        delete pool;
        pool = nullptr;
    }
}

QString XStitcher::Init(){
    // read setting file -----------------------------------------------------------------------
    QSettings *configIniRead = new QSettings("./SendUdpConfig.ini", QSettings::IniFormat);
    configIniRead->setIniCodec("utf-8");
    rtsp_port = configIniRead->value("Code/RtspPort").toString().toStdString();

    // open cameras via RTSP
    char cam_name[20];
    for(int i = 1; i <= 16; i++){
        sprintf(cam_name,"Cameras/cam%d", i);
        Cameras.emplace_back(
              configIniRead->value(cam_name).toString().toStdString()
              );
      }
    std::cout << "camera1: " << Cameras[0] << std::endl;

    delete configIniRead;
    // -------------------------------------------------------------

    // open cameras
      cap1.open(Cameras[0]);
      cap2.open(Cameras[1]);
      cap3.open(Cameras[2]);
      cap4.open(Cameras[3]);
      // prepare data
      for(int i = 0; i < 4; i++){
          processedImgs.emplace_back(cv::Mat(Size(960, 540), CV_8UC3));
        }

    // init rtsp -----------------
    mrtsp->Init(rtsp_port);
    YuvImg.create(Size(1920, 1080), CV_8UC3);
    // ---------------------------

    // extract features ---------------------------------------
    Mat temp1, temp2, temp3, temp4, tmp;
    cap1.read(tmp);
//    imshow("tmp", tmp);
//    cv::waitKey();
    resize(tmp, temp1, Size(), 0.5, 0.5);
    cap2.read(tmp);
    resize(tmp, temp2, Size(), 0.5, 0.5);
    cap3.read(tmp);
    resize(tmp, temp3, Size(), 0.5, 0.5);
    cap4.read(tmp);
    resize(tmp, temp4, Size(), 0.5, 0.5);

    homo1 = ExtractAndCalc(temp1, temp2);
    homo2 = ExtractAndCalc(temp3, temp4);

    up1920.create(Size(1920, 540), CV_8UC3);
    down1920.create(Size(1920, 1080), CV_8UC3);

    // calc corners
    CalcCorners(homo1, temp1, corners1);
  //  cout << "left_top:" << corners1.left_top << endl;
  //  cout << "left_bottom:" << corners1.left_bottom << endl;
  //  cout << "right_top:" << corners1.right_top << endl;
  //  cout << "right_bottom:" << corners1.right_bottom << endl;
    CalcCorners(homo2, temp3, corners2);
  //  cout << "left_top:" << corners2.left_top << endl;
  //  cout << "left_bottom:" << corners2.left_bottom << endl;
  //  cout << "right_top:" << corners2.right_top << endl;
  //  cout << "right_bottom:" << corners2.right_bottom << endl;

    Mat imageTransform1, imageTransform2;
    warpPerspective(temp2, imageTransform1, homo1, Size(MAX(corners1.right_top.x, corners1.right_bottom.x), temp2.rows));
    warpPerspective(temp4, imageTransform2, homo2, Size(MAX(corners2.right_top.x, corners2.right_bottom.x), temp4.rows));

    // initialize storage area
    uparea.create(Size(imageTransform1.cols, imageTransform1.rows), CV_8UC3);
    uparea.setTo(0);
    downarea.create(Size(imageTransform2.cols, imageTransform2.rows), CV_8UC3);
    downarea.setTo(0);
  //  dst.create(Size(MAX(uparea.cols, downarea.cols), uparea.rows + downarea.rows), CV_8UC3);
    dst.create(Size(1920, 1080), CV_8UC3);
    dst.setTo(0);
  //  qDebug() << uparea.cols <<"----"<<uparea.rows <<"----"<<downarea.cols<<"----"<<downarea.rows;
    // extract features end ---------------------------------------

    isInit = true;
    cout << "Init success!\n";

    return QString("test string");
}

bool XStitcher::release(){
    this->~XStitcher();
    return true;
}

void XStitcher::run(){
    if(!isInit){
        cout << "please call the Init() function first! \n";
        return;
    }
    this->Stitch_MultiThread();

}

void XStitcher::CalcCorners(const Mat &H, const Mat &src, four_corners_t &corners){
    double v2[] = { 0, 0, 1 };//左上角
    double v1[3];//变换后的坐标值
    Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量

    V1 = H * V2;
    //左上角(0,0,1)
    cout << "V2: " << V2 << endl;
    cout << "V1: " << V1 << endl;
    corners.left_top.x = v1[0] / v1[2];
    corners.left_top.y = v1[1] / v1[2];

    //左下角(0,src.rows,1)
    v2[0] = 0;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.left_bottom.x = v1[0] / v1[2];
    corners.left_bottom.y = v1[1] / v1[2];

    //右上角(src.cols,0,1)
    v2[0] = src.cols;
    v2[1] = 0;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_top.x = v1[0] / v1[2];
    corners.right_top.y = v1[1] / v1[2];

    //右下角(src.cols,src.rows,1)
    v2[0] = src.cols;
    v2[1] = src.rows;
    v2[2] = 1;
    V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
    V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
    V1 = H * V2;
    corners.right_bottom.x = v1[0] / v1[2];
    corners.right_bottom.y = v1[1] / v1[2];
}

void XStitcher::OptimizeSeam(Mat &img1, Mat &trans, Mat &dst, four_corners_t& corners){
  int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界

  //int start = (corners.left_top.x + corners.left_bottom.x) / 2;


  double processWidth = img1.cols - start;//重叠区域的宽度
  int rows = dst.rows;
  int cols = img1.cols; //注意，是列数*通道数
  double alpha = 1;//img1中像素的权重

  //qDebug()<< "width: " << (cols - start) << " total cols: " << cols << endl;
  for (int i = 0; i < rows; i++)
  {
      uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
      uchar* t = trans.ptr<uchar>(i);
      uchar* d = dst.ptr<uchar>(i);


      // reduce the total pixels to process
//      for (int j = (cols - start) >= 0.2*cols ? (start + cols) / 2 : start; j < cols; j++)
      for (int j = start; j < cols; j++){
          //c如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
          if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0){
              alpha = 1;
          }
          else if(p[j * 3] == 0 && p[j * 3 + 1] == 0 && p[j * 3 + 2] == 0){
              alpha = 0;
            }
          else{
              //img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好
              alpha = (processWidth - (j - start)) / processWidth;
          }

          d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
          d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
          d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);

      }
  }

}

Mat XStitcher::ExtractAndCalc(Mat& image01, Mat& image02){
//  Mat image01 = imread(img1, 1);
//  Mat image02 = imread(img2, 1);
//    namedWindow("p2", 0);
//    namedWindow("p1", 0);
//    imshow("p1", image01);  //left
//    imshow("p2", image02);  //right

  //灰度图转换
  Mat image1, image2;
  if(image01.channels() == 3)
    cvtColor(image01, image1, COLOR_RGB2GRAY);
  if(image02.channels() == 3)
    cvtColor(image02, image2, COLOR_RGB2GRAY);

  //提取特征点,计算特征向量
  int minHessian = 800;
  Ptr<xfeatures2d::SURF>detector = xfeatures2d::SURF::create(minHessian);
  Mat descriptors1, descriptors2;
  vector<KeyPoint> keyPoint1, keyPoint2;
  detector->detectAndCompute(image1,noArray(),keyPoint1,descriptors1);
  detector->detectAndCompute(image2,noArray(),keyPoint2,descriptors2);

  FlannBasedMatcher matcher;
  vector<vector<DMatch> > matchePoints;
  vector<DMatch> GoodMatchePoints;

  vector<Mat> train_desc(1, descriptors1);
  matcher.add(train_desc);
  matcher.train();

  matcher.knnMatch(descriptors2, matchePoints, 2);
  cout << "total match points: " << matchePoints.size() << endl;

  // Lowe's algorithm,获取优秀匹配点
  for (int i = 0; i < matchePoints.size(); i++)
  {
      if (matchePoints[i][0].distance < 0.6 * matchePoints[i][1].distance)
      {
          GoodMatchePoints.push_back(matchePoints[i][0]);
      }
  }

  Mat first_match;
   drawMatches(image02, keyPoint2, image01, keyPoint1, GoodMatchePoints, first_match);
//     drawMatches(image01, keyPoint1, image02, keyPoint2, GoodMatchePoints, first_match);
//  imshow("first_match ", first_match);
  imwrite("first_match.jpg", first_match);
  //waitKey();


  /*
  图像配准
  这样子我们就可以得到了两幅待拼接图的匹配点集，接下来我们进行图像的配准，即将两张图像转换为同一坐标下，
  这里我们需要使用findHomography函数来求得变换矩阵。但是需要注意的是，findHomography函数所要用到的
  点集是Point2f类型的，所有我们需要对我们刚得到的点集GoodMatchePoints再做一次处理，使其转换为Point2f
  类型的点集。
  */
  vector<Point2f> imagePoints1, imagePoints2;

  for (int i = 0; i<GoodMatchePoints.size(); i++)
  {
      imagePoints2.push_back(keyPoint2[GoodMatchePoints[i].queryIdx].pt);
      imagePoints1.push_back(keyPoint1[GoodMatchePoints[i].trainIdx].pt);
  }

  /*这样子，我们就可以拿着imagePoints1, imagePoints2去求变换矩阵了，并且实现图像配准。
   * 值得注意的是findHomography函数的参数中我们选泽了CV_RANSAC，这表明我们选择RANSAC算法继续
   * 筛选可靠地匹配点，这使得匹配点解更为精确*/
  //获取图像1到图像2的投影映射矩阵 尺寸为3*3
  Mat homo = findHomography(imagePoints2, imagePoints1, cv::RANSAC);
  ////也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差
  //Mat   homo=getPerspectiveTransform(imagePoints1,imagePoints2);
  cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵

  //waitKey();

  return homo;
}


// get one frame from camera and finish the preprocess stage, return 960x540 RGB
bool XStitcher::preprocessThread(int capIdx, Mat &output, double fx, double fy, bool cylindProjection){

  assert(fx != 0 && fy != 0);

  Mat temp;

  switch (capIdx) {
  case 0:
      if(!cap1.read(temp)){
          cap1.release();
          cap1.open(Cameras[0]);
          cout << " restart cam 1\n";
      }
      break;
  case 1:
      if(!cap2.read(temp)){
          cap2.release();
          cap2.open(Cameras[1]);
          cout << " restart cam 2\n";
      }
      break;
  case 2:
      if(!cap3.read(temp)){
          cap3.release();
          cap3.open(Cameras[2]);
          cout << " restart cam 3\n";
      }
      break;
  case 3:
      if(!cap4.read(temp)){
          cap4.release();
          cap4.open(Cameras[3]);
          cout << " restart cam 4\n";
      }
      break;
  default:
      cout << " error cap idx! return...\n";
      return false;
//      break;
  }

  if(temp.empty()){
      return false;
    }

//  auto start = system_clock::now();
  // need or not to cylindricalProjection each frame?
  if(cylindProjection){

      resize(temp, temp, Size(), fx, fy, cv::INTER_LINEAR);

      cylindricalProjection(temp, output);
    }
  else{
      resize(temp, output, Size(), fx, fy, cv::INTER_LINEAR);
    }
//  auto end = system_clock::now();
//  auto duration = duration_cast<microseconds>(end - start);
//  std::cout << "capture and preprocess one frame spends "
//           << double(duration.count()) * microseconds::period::num / microseconds::period::den
//           << "seconds" << endl;

//  qDebug() << output.cols << "  ---  " << output.rows;

  return true;
}

void XStitcher::cylindricalProjection(Mat &imgin, Mat &imgout){
  if (!imgin.data)
          return;
//  imshow("image1", imgin);

  float w = imgin.cols;
  float h = imgin.rows;
  float f = (w / 2) / atan(pi / 8);

  for (int i = 0; i < imgin.rows; i++)
  {
          for (int j = 0; j < imgin.cols; j++)
          {
                  float x = j;
                  float y = i;
                  float x1 = f * atan((x - w / 2) / f) + f * atan(w / (2.0f * f));
                  float y1 = f * (y - h / 2.0f) / sqrt((x - w / 2.0f) * (x - w / 2.0f) + f * f) + h / 2.0f;

                  int col = (int)(x1 + 0.5f);//加0.5是为了四舍五入
                  int row = (int)(y1 + 0.5f);//加0.5是为了四舍五入

                  if (col < imgin.cols && row < imgin.rows)
                  {
                          imgout.at<Vec3b>(row, col)[0] = imgin.at<Vec3b>(i, j)[0];
                          imgout.at<Vec3b>(row, col)[1] = imgin.at<Vec3b>(i, j)[1];
                          imgout.at<Vec3b>(row, col)[2] = imgin.at<Vec3b>(i, j)[2];
                  }
          }
  }

//  imshow("imgout", imgout);
}

// support 4 video / cameras
void XStitcher::Stitch_MultiThread(){

  // process loop
  while(true){

      auto start = system_clock::now();

      // allocate tasks
      for(int i = 0; i < 4; i++){
          std::future<bool> f = pool->submit(
                std::bind(&XStitcher::preprocessThread, this, i, ref(processedImgs[i]), 0.5, 0.5, false
                  ));
          infs.emplace_back(std::move(f));
        }
      for(auto & inf: infs){    //future对象不能复制，所以auto 后面要加& 来表示取引用
          if(inf.wait_for(std::chrono::seconds(1)) == std::future_status::timeout){
              cout << "执行挂起的任务(resize)\n";
              pool->run_pending_task();    // 执行挂起的任务
            }
          inf.wait();
        }

      // stitch uparea
      std::future<bool> f1 = pool->submit(
            std::bind(&XStitcher::warpThread, this, ref(processedImgs[0]), ref(processedImgs[1]),
              ref(uparea), ref(up1920), homo1, corners1));
      flgs.emplace_back(std::move(f1));
      // stitch downarea
      std::future<bool> f2 = pool->submit(
            std::bind(&XStitcher::warpThread, this, ref(processedImgs[2]), ref(processedImgs[3]),
              ref(downarea), ref(down1920), homo2, corners2));
      flgs.emplace_back(std::move(f2));

      for(auto & flg: flgs){    //future对象不能复制，所以auto 后面要加& 来表示取引用
          if(flg.wait_for(std::chrono::seconds(1)) == std::future_status::timeout){
              cout << "执行挂起的任务(resize)\n";
              pool->run_pending_task();    // 执行挂起的任务
            }
          flg.wait();
        }


      uparea.copyTo(dst(Rect((1920 - uparea.cols)/2, 0, uparea.cols, uparea.rows)));
      downarea.copyTo(dst(Rect((1920 - downarea.cols)/2, uparea.rows, downarea.cols, downarea.rows)));
//      up1920.copyTo(dst(Rect(0, 0, up1920.cols, up1920.rows)));
//      down1920.copyTo(dst(Rect(0, up1920.rows, down1920.cols, down1920.rows)));

      // send frame
      cvtColor(dst, YuvImg, COLOR_BGR2YUV_I420);
      mrtsp->sendFrameThread(YuvImg);

      auto end = system_clock::now();
      auto duration = duration_cast<microseconds>(end - start);
      cout << "one frame total spends "
               << double(duration.count()) * microseconds::period::num / microseconds::period::den
               << "seconds" << endl;

      infs.clear();
      flgs.clear();

    }


}

bool XStitcher::warpThread(Mat &img1, Mat& img2, Mat& area, Mat& dst, Mat &homo, four_corners_t& corners){

//  auto start = system_clock::now();

  Mat imgTransform;

  warpPerspective(img2, imgTransform, homo, Size(MAX(corners.right_top.x, corners.right_bottom.x), img2.rows));
  imgTransform.copyTo(area(Rect(0, 0, imgTransform.cols, imgTransform.rows)));
  img1.copyTo(area(Rect(0, 0, img1.cols, img1.rows)));

  OptimizeSeam(img1, imgTransform, area, corners);

//  if(area.cols != 1920 || area.rows != 540){
////      cout << " resize to accordant size" << endl;
//      cv::resize(area, dst, cv::Size(1920, 540));
//    }

//  auto end = system_clock::now();
//  auto duration = duration_cast<microseconds>(end - start);
//  cout << "warp one frame spends "
//           << double(duration.count()) * microseconds::period::num / microseconds::period::den
//           << "seconds" << endl;

  return true;
}
