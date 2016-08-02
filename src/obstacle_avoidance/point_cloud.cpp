#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include "jackal_nav/JackalTimeLog.h"
#include <ctime>
#include <fstream>
#include <string>
#include "elas.h"
#include "image.h"
#include "popt_pp.h"

using namespace std;
using namespace cv;

Mat img1, img2, leftim, rightim, leftim_res, rightim_res;
Mat valid_disp;
Mat R1, R2, P1, P2, Q;
Mat K1, K2, R;
Vec3d T;
Mat D1, D2;
Mat XR, XT, V, pos;
Mat lmapx, lmapy, rmapx, rmapy;

image_transport::Publisher disp_pub;
ros::Publisher pcl_publisher;
ros::Publisher obstacle_scan_publisher;
ros::Publisher marker_pub;
ros::Publisher time_log_publisher;

jackal_nav::JackalTimeLog time_log;

Size rawimsize;
int im_width = 640;
int im_height = 400;
int crop_offset_x = 0;
int crop_offset_y = 0;
int crop_im_width = 640;
int crop_im_height = 400;
const int INF = 1e9;
uint32_t seq = 0;

char* calib_file;
char* dmap_time_file;
char* pcl_time_file;
char* obst_scan_time_file;
bool logging = false;
bool gen_pcl = false;

const double GP_HEIGHT_THRESH = 0.04; // group plane height threshold
const double GP_ANGLE_THRESH = 4. * 3.1415 / 180.; // ground plane angular height threshold
const double GP_DIST_THRESH = 1.0; // starting distance for angular threshold
const double ROBOT_HEIGHT = 0.34;

bool inImg(int x, int y) {
  // check if pixel lies inside image
  if (x >= 0 && x < leftim_res.cols && y >= 0 && y < leftim_res.rows)
    return true;
}

void cacheDisparityValues() {
  valid_disp = Mat(crop_im_height, crop_im_width, CV_8UC2, Scalar(255,3));
  for (int i = 0; i < crop_im_width; i++) {
    for (int j = 0; j < crop_im_height; j++) {
      int d;
      for (d = 3; d <= 255; d++) {
        V.at<double>(0,0) = (double)(i + crop_offset_x);
        V.at<double>(1,0) = (double)(j + crop_offset_y);
        V.at<double>(2,0) = (double)d;
        V.at<double>(3,0) = 1.;
        pos = Q * V;
        double X = pos.at<double>(0,0) / pos.at<double>(3,0);
        double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
        double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
        Mat point3d_cam = Mat(3, 1, CV_64FC1);
        point3d_cam.at<double>(0,0) = X;
        point3d_cam.at<double>(1,0) = Y;
        point3d_cam.at<double>(2,0) = Z;
        Mat point3d_robot = XR * point3d_cam + XT;
        X = point3d_robot.at<double>(0,0);
        Y = point3d_robot.at<double>(1,0);
        Z = point3d_robot.at<double>(2,0);
        if (Z > ROBOT_HEIGHT || Z < 0.) {
          continue;
        }
        if (X < GP_DIST_THRESH) {
          if (Z < GP_HEIGHT_THRESH)
            continue;
        } else {
          if (Z < GP_HEIGHT_THRESH + tan(GP_ANGLE_THRESH) * (X - GP_DIST_THRESH))
            continue;
        }
        break;
      }
      valid_disp.at<Vec2b>(j,i)[0] = d;
      valid_disp.at<Vec2b>(j,i)[1] = 255;
    }
  }
  cout << "Disparity values cached!" << endl;
}

void publishObstacleScan(vector< Point3d > points, uint32_t seq) {
  clock_t begin = clock();
  double fov = 90.; // FOV for obstacle scan
  int bin_size = 90; // Max number of obstacle points
  double min_angle = 400, max_angle = -400;
  double range_min = INF, range_max = -500;
  double scan[bin_size];
  sensor_msgs::LaserScan obstacle_scan;
  obstacle_scan.header.seq = seq;
  obstacle_scan.header.frame_id = "jackal";
  obstacle_scan.header.stamp = ros::Time::now();
  for (int i = 0; i < bin_size; i++) {
    scan[i] = INF;
  }
  for (int i = 0; i < points.size(); i++) {
    if (points[i].x < GP_DIST_THRESH) {
      if (points[i].z < GP_HEIGHT_THRESH)
        continue;
    } else {
      if (points[i].z < GP_HEIGHT_THRESH + tan(GP_ANGLE_THRESH) * (points[i].x - GP_DIST_THRESH))
        continue;
    }
    double theta_rad = atan2(points[i].y, points[i].x);
    double theta_deg = theta_rad * 180. / 3.1415;
    min_angle = min(min_angle, theta_rad);
    max_angle = max(max_angle, theta_rad);
    double r = sqrt(points[i].y*points[i].y + points[i].x*points[i].x);
    range_max = max(range_max, r);
    range_min = min(range_min, r);
    int j = floor((double)bin_size * (fov / 2. - theta_deg) / fov);
    if (r < scan[j]) {
      scan[j] = r;
    }
  }
  obstacle_scan.angle_min = min_angle;
  obstacle_scan.angle_max = max_angle;
  obstacle_scan.range_min = range_min;
  obstacle_scan.range_max = range_max;
  obstacle_scan.angle_increment = 3.1415 / 180.;
  obstacle_scan.scan_time = 0.001;
  obstacle_scan.time_increment = 0.1;
  for (int i = bin_size-1; i >= 0; i--) {
    if (scan[i] < INF-1) {
      obstacle_scan.ranges.push_back(scan[i]);
    }
  }
  obstacle_scan_publisher.publish(obstacle_scan);
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  if (logging) {
    ofstream myfile;
    myfile.open(obst_scan_time_file, ios::out | ios::app);
    myfile << elapsed_secs << endl;
    myfile.close();

    time_log.obstacle_scan_time = elapsed_secs;
    time_log_publisher.publish(time_log);
  }
}

void publishObstacleScan(Mat& show, uint32_t seq) {
  clock_t begin = clock();

  double fov = 90.; // FOV for obstacle scan
  int bin_size = 90; // Max number of obstacle points
  double min_angle = 400, max_angle = -400;
  double range_min = INF, range_max = -500;
  double scan[bin_size];
  sensor_msgs::LaserScan obstacle_scan;
  obstacle_scan.header.seq = seq;
  obstacle_scan.header.frame_id = "jackal";
  obstacle_scan.header.stamp = ros::Time::now();
  for (int i = 0; i < bin_size; i++) {
    scan[i] = INF;
  }
  for (int i = 0; i < leftim_res.cols; i++) {
    for (int j = 0; j < leftim_res.rows; j++) {
      int d = show.at<uchar>(j,i);
      if (d < valid_disp.at<Vec2b>(j,i)[0] || d > valid_disp.at<Vec2b>(j,i)[1])
        continue;
      V.at<double>(0,0) = (double)(i + crop_offset_x);
      V.at<double>(1,0) = (double)(j + crop_offset_y);
      V.at<double>(2,0) = (double)d;
      V.at<double>(3,0) = 1.;
      pos = Q * V;
      double X = pos.at<double>(0,0) / pos.at<double>(3,0);
      double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
      double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
      Mat point3d_cam = Mat(3, 1, CV_64FC1);
      point3d_cam.at<double>(0,0) = X;
      point3d_cam.at<double>(1,0) = Y;
      point3d_cam.at<double>(2,0) = Z;
      Mat point3d_robot = XR * point3d_cam + XT;
      X = point3d_robot.at<double>(0,0);
      Y = point3d_robot.at<double>(1,0);
      Z = point3d_robot.at<double>(2,0);
      double theta_rad = atan2(Y, X);
      double theta_deg = theta_rad * 180. / 3.1415;
      min_angle = min(min_angle, theta_rad);
      max_angle = max(max_angle, theta_rad);
      double r = sqrt(Y*Y + X*X);
      range_max = max(range_max, r);
      range_min = min(range_min, r);
      int k = floor((double)bin_size * (fov / 2. - theta_deg) / fov);
      if (r < scan[k]) {
        scan[k] = r;
      }
    }
  }
  obstacle_scan.angle_min = min_angle;
  obstacle_scan.angle_max = max_angle;
  obstacle_scan.range_min = range_min;
  obstacle_scan.range_max = range_max;
  obstacle_scan.angle_increment = 3.1415 / 180.;
  obstacle_scan.scan_time = 0.001;
  obstacle_scan.time_increment = 0.1;
  for (int i = bin_size-1; i >= 0; i--) {
    if (scan[i] < INF-1) {
      obstacle_scan.ranges.push_back(scan[i]);
    }
  }
  obstacle_scan_publisher.publish(obstacle_scan);
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

  if (logging && !gen_pcl) {
    ofstream myfile;
    myfile.open(obst_scan_time_file, ios::out | ios::app);
    myfile << elapsed_secs << endl;
    myfile.close();

    time_log.obstacle_scan_time = elapsed_secs;
    time_log_publisher.publish(time_log);
  }
}

void publishPointCloud(Mat& show, uint32_t seq) {
  if (!gen_pcl) {
    publishObstacleScan(show, seq);
    return;
  }

  clock_t begin = clock();

  vector< Point3d > points;
  sensor_msgs::PointCloud pc;
  sensor_msgs::ChannelFloat32 ch;
  ch.name = "rgb";
  pc.header.seq = seq;
  pc.header.frame_id = "jackal";
  pc.header.stamp = ros::Time::now();
  for (int i = 0; i < leftim_res.cols; i++) {
    for (int j = 0; j < leftim_res.rows; j++) {
      int d = show.at<uchar>(j,i);
      if (d < 2) {
        continue;
      }
      V.at<double>(0,0) = (double)(i + crop_offset_x);
      V.at<double>(1,0) = (double)(j + crop_offset_y);
      V.at<double>(2,0) = (double)d;
      V.at<double>(3,0) = 1.;
      pos = Q * V;
      double X = pos.at<double>(0,0) / pos.at<double>(3,0);
      double Y = pos.at<double>(1,0) / pos.at<double>(3,0);
      double Z = pos.at<double>(2,0) / pos.at<double>(3,0);
      Mat point3d_cam = Mat(3, 1, CV_64FC1);
      point3d_cam.at<double>(0,0) = X;
      point3d_cam.at<double>(1,0) = Y;
      point3d_cam.at<double>(2,0) = Z;
      Mat point3d_robot = XR * point3d_cam + XT;
      
      if (point3d_robot.at<double>(2,0) < 0.) {
        continue;
      }
      
      points.push_back(Point3d(point3d_robot));
      geometry_msgs::Point32 pt;
      pt.x = point3d_robot.at<double>(0,0);
      pt.y = point3d_robot.at<double>(1,0);
      pt.z = point3d_robot.at<double>(2,0);
      pc.points.push_back(pt);
      int32_t red, blue, green;
      if (pt.x < GP_DIST_THRESH) {
        if (pt.z < GP_HEIGHT_THRESH) {
          red = 0;
          blue = 0;
          green = 255;
        } else {
          red = leftim_res.at<Vec3b>(j,i)[2];
          green = leftim_res.at<Vec3b>(j,i)[1];
          blue = leftim_res.at<Vec3b>(j,i)[0];
        }
      } else {
        if (pt.z < GP_HEIGHT_THRESH + tan(GP_ANGLE_THRESH) * (pt.x - GP_DIST_THRESH)) {
          red = 0;
          blue = 0;
          green = 255;
        } else {
          red = leftim_res.at<Vec3b>(j,i)[2];
          green = leftim_res.at<Vec3b>(j,i)[1];
          blue = leftim_res.at<Vec3b>(j,i)[0];
        }
      }
      // color point cloud
      int32_t rgb = (red << 16 | green << 8 | blue);
      ch.values.push_back(*reinterpret_cast<float*>(&rgb));
    }
  }
  pc.channels.push_back(ch);
  pcl_publisher.publish(pc);
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  if (logging && gen_pcl) {
    time_log.header.frame_id = "jackal";
    time_log.header.seq = seq;
    time_log.header.stamp = ros::Time::now();
    ofstream myfile;
    myfile.open(pcl_time_file, ios::out | ios::app);
    myfile << elapsed_secs << endl;
    myfile.close();

    time_log.pcl_time = elapsed_secs;
  }

  publishObstacleScan(points, seq);
}

Mat generateDisparityMap(Mat& left, Mat& right) {
  Mat lb, rb;
  if (left.empty() || right.empty()) 
    return left;
  cvtColor(left, lb, CV_BGR2GRAY);
  cvtColor(right, rb, CV_BGR2GRAY);

  const Size imsize = lb.size();
  const int32_t dims[3] = {imsize.width,imsize.height,imsize.width}; // bytes per line = width

  Mat leftdpf = Mat::zeros(imsize, CV_32F);
  Mat rightdpf = Mat::zeros(imsize, CV_32F);

  Elas::parameters param;
  param.postprocess_only_left = true;
  Elas elas(param);
  elas.process(lb.data,rb.data,leftdpf.ptr<float>(0),rightdpf.ptr<float>(0),dims);

  int max_disp = -1;
  for (int i = 0; i < imsize.width; i++) {
    for (int j = 0; j < imsize.height; j++) {
      if (leftdpf.at<uchar>(j,i) > max_disp) max_disp = leftdpf.at<uchar>(j,i);
    }
  }
  for (int i = 0; i < imsize.width; i++) {
    for (int j = 0; j < imsize.height; j++) {
      leftdpf.at<uchar>(j,i) = (int)max(255.0*(float)leftdpf.at<uchar>(j,i)/max_disp,0.0);
    }
  }
  Mat show = Mat(crop_im_height, crop_im_width, CV_8UC1, Scalar(0));
  leftdpf.convertTo(show, CV_8U, 1.);
  if (!show.empty()) {
    sensor_msgs::ImagePtr disp_msg;
    disp_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", show).toImageMsg();
    disp_pub.publish(disp_msg);
  }
  return show;
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
    
    clock_t begin = clock();

    Mat dmap = generateDisparityMap(leftim_res, rightim_res);

    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    if (logging) {
      time_log.header.frame_id = "jackal";
      time_log.header.seq = seq;
      time_log.header.stamp = ros::Time::now();
      ofstream myfile;
      myfile.open(dmap_time_file, ios::out | ios::app);
      myfile << elapsed_secs << endl;
      myfile.close();

      time_log.dmap_time = elapsed_secs;
    }

    if (!dmap.empty())
      publishPointCloud(dmap, seq);
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
  ros::init(argc, argv, "jackal_obstacle_avoidance");
  ros::NodeHandle nh;
  
  static struct poptOption options[] = {
    { "img-height",'h',POPT_ARG_INT,&crop_im_height,0,"Image height","NUM" },
    { "calib-file",'c',POPT_ARG_STRING,&calib_file,0,"Stereo calibration file","STR" },
    { "logging",'l',POPT_ARG_NONE,&logging,0,"Log pipeline time","NONE" },
    { "gen-pcl",'g',POPT_ARG_NONE,&gen_pcl,0,"Generate PCL","NONE" },
    { "dmap-file",'d',POPT_ARG_STRING,&dmap_time_file,0,"DMAP time file","STR" },
    { "pcl-file",'p',POPT_ARG_STRING,&pcl_time_file,0,"PCL time file","STR" },
    { "scan-file",'s',POPT_ARG_STRING,&obst_scan_time_file,0,"Scan time file","STR" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  image_transport::ImageTransport it(nh);
  disp_pub = it.advertise("/webcam/left/depth_map", 1);
  pcl_publisher = nh.advertise<sensor_msgs::PointCloud>("/webcam/left/point_cloud", 1);
  obstacle_scan_publisher = nh.advertise<sensor_msgs::LaserScan>("/webcam/left/obstacle_scan", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  if (logging) {
    time_log_publisher = nh.advertise<jackal_nav::JackalTimeLog>("/jackal/time_log", 1);
  }
  
  cv::FileStorage fs1(calib_file, cv::FileStorage::READ);
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

  V = Mat(4, 1, CV_64FC1);
  pos = Mat(4, 1, CV_64FC1);

  rawimsize = Size(im_width, im_height);
  img1 = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));
  img2 = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));
  rightim = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));

  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);

  if (!gen_pcl)
    cacheDisparityValues();

  ros::Subscriber subl = nh.subscribe("/camera_left/image_color/compressed", 1, imageCallbackLeft);
  ros::Subscriber subr = nh.subscribe("/camera_right/image_color/compressed", 1, imageCallbackRight);

  ros::spin();
}
