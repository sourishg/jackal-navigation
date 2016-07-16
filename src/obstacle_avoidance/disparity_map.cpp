#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>
#include <fstream>
#include <string>

using namespace std;
using namespace cv;

Mat img1, img2;
Mat R1, R2, P1, P2, Q;
Mat K1, K2, R;
Vec3d T;
Mat D1, D2;
Mat XR, XT, V, pos;
Mat lmapx, lmapy, rmapx, rmapy;

Size rawimsize;
int im_width = 270;
int im_height = 180;
int crop_offset_x = 0;
int crop_offset_y = 0;
int crop_im_width = 270;
int crop_im_height = 180;
const int INF = 1e9;
string working_dir;

bool inImg(int x, int y) {
  // check if pixel lies inside image
  if (x >= 0 && x < show.cols && y >= 0 && y < show.rows)
    return true;
}

void load_images(Mat& img_left, Mat& img_right) {
  Mat tmp1, tmp2;
  string left_file = working_dir;
  left_file.append("data/imgs/left1.jpg");
  tmp1 = imread(left_file, 1);
  string right_file = working_dir;
  right_file.append("data/imgs/right1.jpg");
  tmp2 = imread(right_file, 1);
  cv::remap(tmp1, img_left, lmapx, lmapy, cv::INTER_LINEAR);
  cv::remap(tmp2, img_right, rmapx, rmapy, cv::INTER_LINEAR);
}

int main(int argc, char **argv)
{
  cv::namedWindow("view_left");
  cv::namedWindow("view_right");
  cv::namedWindow("view_disp");
  working_dir = argv[1];
  string calib_file = working_dir;
  calib_file.append("src/calibration/stereo_calib.yml");
  cv::FileStorage fs1(calib_file.c_str(), cv::FileStorage::READ);
  fs1["K1"] >> K1;
  fs1["K2"] >> K2;
  fs1["D1"] >> D1;
  fs1["D2"] >> D2;
  fs1["R"] >> R;
  fs1["T"] >> T;

  fs1["R1"] >> R1;
  fs1["R2"] >> R2;
  fs1["P1"] >> P1;
  fs1["P2"] >> P2;
  fs1["Q"] >> Q;
  fs1["XR"] >> XR;
  fs1["XT"] >> XT;

  rawimsize = Size(im_width, im_height);
  img1 = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));
  img2 = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));

  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);

  load_images(img1, img2);

  while (1) {
    imshow("view_left", img1);
    imshow("view_right", img2);
    if (waitKey(0) > 0) {
      break;
    }
  }
  
  cv::destroyWindow("view_left");
  cv::destroyWindow("view_right");
  cv::destroyWindow("view_disp");
}
