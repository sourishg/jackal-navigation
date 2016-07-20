#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
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
  if (x >= 0 && x < img1.cols && y >= 0 && y < img1.rows)
    return true;
}

void load_images(Mat& img_left, Mat& img_right) {
  Mat tmp1, tmp2;
  string left_file = working_dir;
  left_file.append("data/imgs/left15.jpg");
  tmp1 = imread(left_file, 1);
  string right_file = working_dir;
  right_file.append("data/imgs/right15.jpg");
  tmp2 = imread(right_file, 1);
  cv::remap(tmp1, img_left, lmapx, lmapy, cv::INTER_LINEAR);
  cv::remap(tmp2, img_right, rmapx, rmapy, cv::INTER_LINEAR);
}

void matchFeatures(Mat& left, Mat& right) {
  clock_t begin = clock();

  OrbFeatureDetector detector(500, 1.2f, 8, 15, 0);
  vector< KeyPoint > left_kpts, right_kpts;
  Mat left_desc, right_desc;
  detector.detect(img1, left_kpts);
  detector.compute(img1, left_kpts, left_desc);
  detector.detect(img2, right_kpts);
  detector.compute(img2, right_kpts, right_desc);
  //drawKeypoints(left, left_kpts, left, Scalar(255,0,0));
  //drawKeypoints(right, right_kpts, right, Scalar(255,0,0));
  cv::Mat results;
  cv::Mat dists;
  int k=2; // find the 2 nearest neighbors
  if(left_desc.type()==CV_8U)
  {
      // Binary descriptors detected (from ORB or Brief)

      // Create Flann LSH index
      cv::flann::Index flannIndex(right_desc, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);

      // search (nearest neighbor)
      flannIndex.knnSearch(left_desc, results, dists, k, cv::flann::SearchParams() );
  }
  else
  {
      // assume it is CV_32F
      // Create Flann KDTree index
      cv::flann::Index flannIndex(right_desc, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);

      // search (nearest neighbor)
      flannIndex.knnSearch(left_desc, results, dists, k, cv::flann::SearchParams() );
  }

  // Conversion to CV_32F if needed
  if(dists.type() == CV_32S)
  {
      cv::Mat temp;
      dists.convertTo(temp, CV_32F);
      dists = temp;
  }
  //drawKeypoints(img1, img1_keypts, Kimg1, Scalar(255,0,0));
  //drawKeypoints(img2, img2_keypts, Kimg2, Scalar(255,0,0));
  
  float nndrRatio = 0.9;
  vector<Point2f> mpts_1, mpts_2; // Used for homography
  for(unsigned int i=0; i<results.rows; ++i)
  {
      // Check if this descriptor matches with those of the objects
      // Apply NNDR
      if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 && dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
      {
          mpts_1.push_back(left_kpts.at(i).pt);

          mpts_2.push_back(right_kpts.at(results.at<int>(i,0)).pt);
      }
  }
  for (int i = 0; i < mpts_1.size(); i++) {
    circle(left, mpts_1[i], 3, Scalar(255,0,0), 2);
    circle(right, mpts_2[i], 3, Scalar(255,0,0), 2);
  }
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  cout << elapsed_secs << endl;
  cout << mpts_1.size() << endl;
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
  matchFeatures(img1, img2);

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
