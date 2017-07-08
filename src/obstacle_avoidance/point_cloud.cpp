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
#include <dynamic_reconfigure/server.h>
#include <jackal_nav/CamToRobotCalibParamsConfig.h>
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

jackal_nav::CamToRobotCalibParamsConfig config;
Size calib_im_size = Size(640, 360);

image_transport::Publisher disp_pub;
ros::Publisher pcl_publisher;
ros::Publisher obstacle_scan_publisher;
ros::Publisher marker_pub;
ros::Publisher time_log_publisher;

jackal_nav::JackalTimeLog time_log;
// kitti 490x180
Size rawimsize;
int im_width = 320; // output image width and height
int im_height = 180;
int crop_offset_x = 0; // starting x coordinate of disparity map
int crop_offset_y = 0; // starting y coordinate of disparity map
int crop_im_width = 320; // width of disparity map
int crop_im_height = 180; // height of disparity map
const int INF = 1e9;
uint32_t seq = 0;

char* calib_file;
char* dmap_time_file;
char* pcl_time_file;
char* obst_scan_time_file;
bool logging = false;
int calib_robot_to_cam = 0;
int gen_pcl = 0;

const double GP_HEIGHT_THRESH = 0.05; // group plane height threshold
const double GP_ANGLE_THRESH = 4. * 3.1415 / 180.; // ground plane angular height threshold
const double GP_DIST_THRESH = 1.0; // starting distance for angular threshold
const double ROBOT_HEIGHT = 0.34;

bool inImg(int x, int y) {
  // check if pixel lies inside image
  if (x >= 0 && x < leftim_res.cols && y >= 0 && y < leftim_res.rows)
    return true;
}

Mat composeRotationCamToRobot(float x, float y, float z) {
  Mat X = Mat::eye(3, 3, CV_64FC1);
  Mat Y = Mat::eye(3, 3, CV_64FC1);
  Mat Z = Mat::eye(3, 3, CV_64FC1);
  
  X.at<double>(1,1) = cos(x);
  X.at<double>(1,2) = -sin(x);
  X.at<double>(2,1) = sin(x);
  X.at<double>(2,2) = cos(x);

  Y.at<double>(0,0) = cos(y);
  Y.at<double>(0,2) = sin(y);
  Y.at<double>(2,0) = -sin(y);
  Y.at<double>(2,2) = cos(y);

  Z.at<double>(0,0) = cos(z);
  Z.at<double>(0,1) = -sin(z);
  Z.at<double>(1,0) = sin(z);
  Z.at<double>(1,1) = cos(z);
  
  return Z*Y*X;
}

Mat composeTranslationCamToRobot(float x, float y, float z) {
  return (Mat_<double>(3,1) << x, y, z);
}

void cacheDisparityValues() {
  // cache all the range for valid disparity values for each pixel
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
        // ignore points below ground plane
        if (Z < 0.) {
          continue;
        }
        // apply ground plane threshold, the ground plane rises
        // at an angle after a distance = GP_DIST_THRESH
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
  Point3d closest_pt[bin_size];
  sensor_msgs::LaserScan obstacle_scan;
  obstacle_scan.header.seq = seq;
  obstacle_scan.header.frame_id = "jackal";
  obstacle_scan.header.stamp = ros::Time::now();
  for (int i = 0; i < bin_size; i++) {
    scan[i] = INF;
  }
  for (int i = 0; i < points.size(); i++) {
    // ignore if ground plane
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
      closest_pt[j] = points[i];
    }
  }
  // publish obstacle scan
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

void publishObstacleScan(Mat& dmap, uint32_t seq) {
  // generate obstacle scan directly from disparity map
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
  // initialize laser scan points to infinity
  for (int i = 0; i < bin_size; i++) {
    scan[i] = INF;
  }
  for (int i = 0; i < leftim_res.cols; i++) {
    for (int j = 0; j < leftim_res.rows; j++) {
      int d = dmap.at<uchar>(j,i);
      // check if disparity values are invalid
      if (d < valid_disp.at<Vec2b>(j,i)[0] || d > valid_disp.at<Vec2b>(j,i)[1])
        continue;
      // do 3D reconstruction
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
      // transfrom from camera frame to robot frame
      Mat point3d_robot = XR * point3d_cam + XT;
      X = point3d_robot.at<double>(0,0);
      Y = point3d_robot.at<double>(1,0);
      Z = point3d_robot.at<double>(2,0);
      // calculate angle of obstacle point
      double theta_rad = atan2(Y, X);
      double theta_deg = theta_rad * 180. / 3.1415;
      min_angle = min(min_angle, theta_rad);
      max_angle = max(max_angle, theta_rad);
      // calculate distance of obstacle point
      double r = sqrt(Y*Y + X*X);
      range_max = max(range_max, r);
      range_min = min(range_min, r);
      // bin the obstacle points based on the angle
      int k = floor((double)bin_size * (fov / 2. - theta_deg) / fov);
      if (r < scan[k]) {
        scan[k] = r;
      }
    }
  }
  // publish obstacle scan
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

void publishPointCloud(Mat& dmap, uint32_t seq) {
  // if not generating full point cloud, then generate obstacle scan directly
  // from disparity map
  if (!gen_pcl) {
    publishObstacleScan(dmap, seq);
    return;
  }
  if (calib_robot_to_cam) {
    XR = composeRotationCamToRobot(config.PHI_X,config.PHI_Y,config.PHI_Z);
    XT = 
composeTranslationCamToRobot(config.TRANS_X,config.TRANS_Y,config.TRANS_Z);
    cout << "Rotation matrix: " << XR << endl;
    cout << "Translation matrix: " << XT << endl;
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
      int d = dmap.at<uchar>(j,i);
      // if low disparity, then ignore
      if (d < 2) {
        continue;
      }
      // V is the vector to be multiplied to Q to get
      // the 3D homogenous coordinates of the image point
      V.at<double>(0,0) = (double)(i + crop_offset_x);
      V.at<double>(1,0) = (double)(j + crop_offset_y);
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
      // ignore points below the ground plane
      /*
      if (point3d_robot.at<double>(2,0) < 0.) {
        continue;
      }
      */
      points.push_back(Point3d(point3d_robot));
      geometry_msgs::Point32 pt;
      pt.x = point3d_robot.at<double>(0,0);
      pt.y = point3d_robot.at<double>(1,0);
      pt.z = point3d_robot.at<double>(2,0);
      pc.points.push_back(pt);
      int32_t red, blue, green;
      // color point cloud and ground plane accordingly
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
      red = leftim_res.at<Vec3b>(j,i)[2];
      green = leftim_res.at<Vec3b>(j,i)[1];
      blue = leftim_res.at<Vec3b>(j,i)[0];
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
  if (left.empty() || right.empty()) 
    return left;
  
  const Size imsize = left.size();
  const int32_t dims[3] = {imsize.width,imsize.height,imsize.width}; // bytes per line = width

  Mat leftdpf = Mat::zeros(imsize, CV_32F);
  Mat rightdpf = Mat::zeros(imsize, CV_32F);

  Elas::parameters param;
  param.postprocess_only_left = true;
  Elas elas(param);
  elas.process(left.data,right.data,leftdpf.ptr<float>(0),rightdpf.ptr<float>(0),dims);

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
    //Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_GRAYSCALE);
    if (tmp.empty())
      return;
    //resize(tmp, img1, rawimsize);
    cv::remap(tmp, leftim, lmapx, lmapy, cv::INTER_LINEAR);
    //leftim = tmp.clone();
    leftim_res = leftim(Rect(crop_offset_x, crop_offset_y, crop_im_width, crop_im_height));
    //imshow("left",leftim_res);
    //waitKey(30);
      
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
    //Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;
    Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_GRAYSCALE);
    if (tmp.empty()) return;
    //resize(tmp, img2, rawimsize);
    cv::remap(tmp, rightim, rmapx, rmapy, cv::INTER_LINEAR);
    //rightim = tmp.clone();
    rightim_res = rightim(Rect(crop_offset_x, crop_offset_y, crop_im_width, crop_im_height));
    //imshow("right", rightim_res);
    //waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

void paramsCallback(jackal_nav::CamToRobotCalibParamsConfig &conf, uint32_t 
level) {
  config = conf;
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
    { "calib-extrinsic",'m',POPT_ARG_NONE,&calib_robot_to_cam,0,
"Calibrate extrinsics between left camera and robot","NONE" },
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
  fs1["XR"] >> XR; // rotation from camera frame to robot frame
  fs1["XT"] >> XT; // translation from camera frame to robot frame

  rawimsize = Size(im_width, im_height);
  Rect validRoi[2];
  cout << "starting rectification" << endl;
  stereoRectify(K1, D1, K2, D2, calib_im_size, R, Mat(T), R1, R2, P1, P2, Q, 
                CV_CALIB_ZERO_DISPARITY, 0, rawimsize, &validRoi[0], &validRoi[1]);
  cout << "done rectification" << endl;

  V = Mat(4, 1, CV_64FC1);
  pos = Mat(4, 1, CV_64FC1);

  rightim = Mat(rawimsize, CV_8UC3, Scalar(0,0,0));

  // undistort and rectify both images
  cv::initUndistortRectifyMap(K1, D1, R1, P1, rawimsize, CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, rawimsize, CV_32F, rmapx, rmapy);

  // if not generating full point cloud, cache disparity values
  if (!gen_pcl)
    cacheDisparityValues();
  
  if (calib_robot_to_cam) {
    dynamic_reconfigure::Server<jackal_nav::CamToRobotCalibParamsConfig> server;
    dynamic_reconfigure::Server<jackal_nav::CamToRobotCalibParamsConfig>::
CallbackType f;
    f = boost::bind(&paramsCallback, _1, _2);
    server.setCallback(f);
  }
  
  // subscribe to camera topics
  ros::Subscriber subl = nh.subscribe("/webcam/left/image_raw/compressed", 1, imageCallbackLeft);
  ros::Subscriber subr = nh.subscribe("/webcam/right/image_raw/compressed", 1, imageCallbackRight);

  ros::spin();
}
