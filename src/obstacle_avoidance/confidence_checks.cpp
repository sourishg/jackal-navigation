#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CompressedImage.h>
#include "jackal_nav/JackalTimeLog.h"
#include <ctime>
#include <fstream>
#include <string>
#include "popt_pp.h"

using namespace std;
using namespace cv;

Mat img1, img2, leftim, rightim, leftim_res, rightim_res;
Mat valid_disp;
Mat R1, R2, P1, P2, Q;
Mat K1, K2, R;
Vec3d T;
Mat D1, D2;
Mat XR, XT;
Mat lmapx, lmapy, rmapx, rmapy;
Mat img_left_desc, img_right_desc;
vector< KeyPoint > kpl, kpr;
int w = 1;
vector< pair< Point, Point > > waypoint_coords;

image_transport::Publisher conf_neg_pub;
ros::Publisher time_log_publisher;

jackal_nav::JackalTimeLog time_log;

Size rawimsize;
int im_width = 320; // change image width and height according to your calibration
int im_height = 180;
int crop_offset_x = 0; // starting x coordinate of disparity map
int crop_offset_y = 0; // starting y coordinate of disparity map
int crop_im_width = 320; // width of disparity map
int crop_im_height = 180; // height of disparity map
const int INF = 1e9;
uint32_t seq = 0;

char* calib_file;
bool logging = false;

const double GP_HEIGHT_THRESH = 0.07; // group plane height threshold
const double GP_ANGLE_THRESH = 4. * 3.1415 / 180.; // ground plane angular height threshold
const double GP_DIST_THRESH = 1.0; // starting distance for angular threshold
const double ROBOT_HEIGHT = 0.34;

bool inImg(int x, int y) {
  // check if pixel lies inside image
  if (x >= 0 && x < leftim_res.cols && y >= 0 && y < leftim_res.rows)
    return true;
}

bool isLeftKeyPoint(int i, int j) {
  int n = kpl.size();
  return (i >= kpl[0].pt.x && i <= kpl[n-1].pt.x
          && j >= kpl[0].pt.y && j <= kpl[n-1].pt.y);
}

bool isRightKeyPoint(int i, int j) {
  int n = kpr.size();
  return (i >= kpr[0].pt.x && i <= kpr[n-1].pt.x
          && j >= kpr[0].pt.y && j <= kpr[n-1].pt.y);
}

long descCost(Point leftpt, Point rightpt, int w) {
  int x0r = kpr[0].pt.x;
  int y0r = kpr[0].pt.y;
  int ynr = kpr[kpr.size()-1].pt.y;
  int x0l = kpl[0].pt.x;
  int y0l = kpl[0].pt.y;
  int ynl = kpl[kpl.size()-1].pt.y;
  long cost = 0;
  for (int j = -w; j <= w; j++) {
    for (int k = -w; k <= w; k++) {
      if (!isLeftKeyPoint(leftpt.x+j, leftpt.y+k) || 
          !isRightKeyPoint(rightpt.x+j, rightpt.y+k))
        continue;
      int idxl = (leftpt.x+j-x0l)*(ynl-y0l+1)+(leftpt.y+k-y0l);
      int idxr = (rightpt.x+j-x0r)*(ynr-y0r+1)+(rightpt.y+k-y0r);
      cost += norm(img_left_desc.row(idxl), img_right_desc.row(idxr), CV_L1);
    }
  }
  return cost / ((2*w+1)*(2*w+1));
}

Point3f get3DCoord(Point p, int d) {
  Mat V = Mat(4, 1, CV_64FC1);
  Mat pos = Mat(4, 1, CV_64FC1);
  // V is the vector to be multiplied to Q to get
  // the 3D homogenous coordinates of the image point
  V.at<double>(0,0) = (double)(p.x);
  V.at<double>(1,0) = (double)(p.y);
  V.at<double>(2,0) = (double)d;
  V.at<double>(3,0) = 1.;
  pos = Q * V; // 3D homogeneous coordinate
  double X = pos.at<double>(0,0) / pos.at<double>(3,0);
  double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
  double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
  Mat point3d_cam = Mat(3, 1, CV_64FC1);
  point3d_cam.at<double>(0,0) = X;
  point3d_cam.at<double>(1,0) = Y;
  point3d_cam.at<double>(2,0) = Z;
  // transform 3D point from camera frame to robot frame
  Mat point3d_robot = XR * point3d_cam + XT;
  Point3f pt3d(point3d_robot.at<double>(0,0), point3d_robot.at<double>(1,0), 
               point3d_robot.at<double>(2,0));
  return pt3d;
}

Point projectPointCam(Point3f p, Mat& P) {
  Mat pt3d = (Mat_<double>(3, 1) << p.x, p.y, p.z);
  Mat pt3d_cam = XR.inv()*(pt3d - XT);
  Mat pt3d_cam_hom = (Mat_<double>(4, 1) << pt3d_cam.at<double>(0,0), 
                      pt3d_cam.at<double>(1,0), pt3d_cam.at<double>(2,0), 1.);
  Mat img_coord = P * pt3d_cam_hom;
  Point imgc;
  imgc.x = img_coord.at<double>(0,0)/img_coord.at<double>(2,0);
  imgc.y = img_coord.at<double>(1,0)/img_coord.at<double>(2,0);
  return imgc;
}

void publishConfNegMatch() {
  Mat conf_neg_img = leftim_res.clone();
  /*
  for (int i = 0; i < waypoint_coords.size(); i++) {
    Point ptl = waypoint_coords[i].first;
    Point ptr = waypoint_coords[i].second;
    long cost2 = descCost(ptl, ptr, w);
    if (cost2 >= 2000) {
      Scalar red = Scalar(0,0,255);
      circle(conf_neg_img, ptl, 1, red, -1, 8, 0);
    }
  }
  */
  if (!conf_neg_img.empty()) {
    sensor_msgs::ImagePtr disp_msg;
    disp_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", conf_neg_img).toImageMsg();
    conf_neg_pub.publish(disp_msg);
  }
}

void cacheDescriptorVals() {
  kpl.clear();
  kpr.clear();
  OrbDescriptorExtractor extractor;
  for (int i = 40; i < leftim_res.cols-40; i++) {
    for (int j = 40; j < leftim_res.rows-40; j++) {
      kpl.push_back(KeyPoint(i,j,1));
      kpr.push_back(KeyPoint(i,j,1));
    }
  }
  extractor.compute(leftim_res, kpl, img_left_desc);
  extractor.compute(rightim_res, kpr, img_right_desc);
}

void cacheWaypointCoords() {
  for (double x = 0.6; x <= 1.8; x += 0.03) {
    for (double y = -0.2; y <= 0.2; y += 0.03) {
      Point3f p(x,y,0.0);
      Point ptl = projectPointCam(p, P1);
      Point ptr = projectPointCam(p, P2);
      waypoint_coords.push_back(make_pair(ptl, ptr));
    }
  }
}

void imageCallbackLeft(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    //Mat tmp = cv_bridge::toCvShare(msg, "mono8")->image;
    Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_UNCHANGED);
    if (tmp.empty()) return;
    resize(tmp, img1, rawimsize);
    cv::remap(img1, leftim, lmapx, lmapy, cv::INTER_LINEAR);
    leftim_res = leftim(Rect(crop_offset_x, crop_offset_y, crop_im_width, crop_im_height));
    if (!leftim_res.empty() && !rightim_res.empty())
      cacheDescriptorVals();
    if (!img_left_desc.empty() && !img_right_desc.empty() && !leftim_res.empty() && !rightim_res.empty())
      publishConfNegMatch();
  }
  catch (cv_bridge::Exception& e)
  {
  }
  seq++;
}

void imageCallbackRight(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    //Mat tmp = cv_bridge::toCvShare(msg, "mono8")->image;
    Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_UNCHANGED);
    if (tmp.empty()) return;
    resize(tmp, img2, rawimsize);
    cv::remap(img2, rightim, rmapx, rmapy, cv::INTER_LINEAR);
    rightim_res = rightim(Rect(crop_offset_x, crop_offset_y, crop_im_width, crop_im_height));
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jackal_confidence_checks");
  ros::NodeHandle nh;

  static struct poptOption options[] = {
    { "win-size",'w',POPT_ARG_INT,&w,0,"Window size","NUM" },
    { "calib-file",'c',POPT_ARG_STRING,&calib_file,0,"Stereo calibration file","STR" },
    { "logging",'l',POPT_ARG_NONE,&logging,0,"Log pipeline time","NONE" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  image_transport::ImageTransport it(nh);
  conf_neg_pub = it.advertise("/webcam/left/conf_neg_match", 1);
  if (logging) {
    // publishes time taken by each step in the pipeline
    time_log_publisher = nh.advertise<jackal_nav::JackalTimeLog>("/jackal/time_log", 1);
  }
  
  cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
  fs1["K1"] >> K1; // left camera matrix
  fs1["K2"] >> K2; // right camera matrix
  fs1["D1"] >> D1; // left camera distortion coeffs
  fs1["D2"] >> D2; // right camera distortion coeffs
  fs1["R"] >> R; // rotation from left to right camera
  fs1["T"] >> T; // translation from left to right camera

  fs1["R1"] >> R1; // rectification transfrom for left camera
  fs1["R2"] >> R2; // rectification transform for right camera
  fs1["P1"] >> P1; // projection matrix in rectified coordinate system for left camera
  fs1["P2"] >> P2; // projection matrix in rectified coordinate system for right camera
  fs1["Q"] >> Q; // depth to disparity mapping matrix
  fs1["XR"] >> XR; // rotation from camera frame to robot frame
  fs1["XT"] >> XT; // translation from camera frame to robot frame

  rawimsize = Size(im_width, im_height);
  img1 = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));
  img2 = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));
  rightim = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));

  // undistort and rectify both images
  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);

  cacheWaypointCoords();

  // subscribe to camera topics
  ros::Subscriber subl = nh.subscribe("/webcam/left/image_raw/compressed", 1, imageCallbackLeft);
  ros::Subscriber subr = nh.subscribe("/webcam/right/image_raw/compressed", 1, imageCallbackRight);

  ros::spin();
}
