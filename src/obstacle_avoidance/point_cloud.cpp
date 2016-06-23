#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <geometry_msgs/Point32.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/pcl_exports.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include "elas.h"
#include "image.h"

using namespace std;
using namespace cv;

Mat img1, img2, leftim, rightim, leftim_res, rightim_res, show;
cv::Mat R1, R2, P1, P2, Q;
cv::Mat K1, K2, R;
cv::Vec3d T;
cv::Mat D1, D2;
Mat XR, XT, V, pos;
cv::Mat lmapx, lmapy, rmapx, rmapy;
Mat leftdisp, rightdisp;
image_transport::Publisher disp_pub;
ros::Publisher pcl_publisher;

int save_count = 1;

Size rawimsize;
Size cropped_imsize;
int im_width = 270;
int im_height = 180;

void publishPointCloud() {
  double fov = 70.;
  int bin_size = 200;
  double min_angle = 400, max_angle = -400;
  double scan[bin_size];
  Point3d closest_pt[bin_size];
  vector< Point3d > points;
  for (int i = 0; i < bin_size; i++) {
    scan[i] = 1e9;
  }
  sensor_msgs::PointCloud pc;
  sensor_msgs::ChannelFloat32 ch;
  ch.name = "rgb";
  pc.header.frame_id = "point_cloud";
  pc.header.stamp = ros::Time();
  for (int i = 0; i < show.cols; i++) {
    for (int j = 0; j < show.rows; j++) {
      int d = show.at<uchar>(j,i);
      if (d < 5)
        continue;
      V.at<double>(0,0) = (double)i;
      V.at<double>(1,0) = (double)j;
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
      points.push_back(Point3d(point3d_robot));
      geometry_msgs::Point32 pt;
      pt.x = point3d_robot.at<double>(0,0);
      pt.y = point3d_robot.at<double>(1,0);
      pt.z = point3d_robot.at<double>(2,0);
      pc.points.push_back(pt);
      int32_t red, blue, green;
      
      if (pt.z < 0.04 && pt.z > -0.04) {
        red = 0;
        blue = 0;
        green = 255;
      } else {
        red = leftim_res.at<Vec3b>(j,i)[2];
        green = leftim_res.at<Vec3b>(j,i)[1];
        blue = leftim_res.at<Vec3b>(j,i)[0];
      }
      int32_t rgb = (red << 16 | green << 8 | blue);
      ch.values.push_back(*reinterpret_cast<float*>(&rgb));
    }
  }
  pc.channels.push_back(ch);
  pcl_publisher.publish(pc);
  /*
  for (int i = 0; i < points.size(); i++) {
    if (points[i].z < 0.1 && points[i].z > -0.1)
      continue;
    double theta = atan2(points[i].y, points[i].x) * 180. / 3.1415;
    double r = sqrt(points[i].y*points[i].y + points[i].x*points[i].x);
    int j = floor((double)bin_size * (fov / 2. - theta) / fov);
    if (r < scan[j]) {
      scan[j] = r;
      closest_pt[j] = points[i];
    }
    if (r < 1) {
      cout << r << endl;
      cout << "Obstacle closer than 50cms." << endl;
    }
  }
  */
}

void imageCallbackLeft(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    //Mat tmp = cv_bridge::toCvShare(msg, "mono8")->image;
    Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_UNCHANGED);
    resize(tmp, img1, rawimsize);
    cv::remap(img1, leftim, lmapx, lmapy, cv::INTER_LINEAR);
    cropped_imsize = Size(270, 180);
    //leftim_res = leftim(Rect(0, 0, 640, 360));
    resize(leftim, leftim_res, cropped_imsize);
    //cv::imshow("view_left", leftim_res);
    Mat lb,rb;
    lb = Mat(cropped_imsize, CV_8UC1, Scalar(0));
    rb = Mat(cropped_imsize, CV_8UC1, Scalar(0));
    if(leftim.channels()==3){cvtColor(leftim_res,lb,CV_BGR2GRAY);}
    else lb=leftim_res;
    if(rightim.channels()==3)cvtColor(rightim_res,rb,CV_BGR2GRAY);
    else rb=rightim_res;
    int bd = 0;
    
    const cv::Size imsize = lb.size();
    const int32_t dims[3] = {imsize.width,imsize.height,imsize.width}; // bytes per line = width

    cv::Mat leftdpf = cv::Mat::zeros(imsize,CV_32F);
    cv::Mat rightdpf = cv::Mat::zeros(imsize,CV_32F);
    
    Elas::parameters param;
    param.postprocess_only_left = false;
    Elas elas(param);
    
    elas.process(lb.data,rb.data,leftdpf.ptr<float>(0),rightdpf.ptr<float>(0),dims);
    //Mat disp;
    //Mat(leftdpf(cv::Rect(bd,0,leftim.cols,leftim.rows))).copyTo(disp);
    //leftdpf.copyTo(leftdisp);
    //disp.convertTo(leftdisp,CV_16S,1);
    //Mat(rightdpf(cv::Rect(bd,0,leftim.cols,leftim.rows))).copyTo(disp);
    //rightdpf.copyTo(disp);
    //disp.convertTo(rightdisp,CV_16S,16);
    
    int max_disp = -1;
    for (int i = 0; i < im_width; i++) {
      for (int j = 0; j < im_height; j++) {
        if (leftdpf.at<uchar>(j,i) > max_disp) max_disp = leftdpf.at<uchar>(j,i);
      }
    }
    for (int i = 0; i < im_width; i++) {
      for (int j = 0; j < im_height; j++) {
        leftdpf.at<uchar>(j,i) = (int)max(255.0*(float)leftdpf.at<uchar>(j,i)/max_disp,0.0);
      }
    }
    
    show = Mat(180, 270, CV_8UC1, Scalar(0));
    leftdpf.convertTo(show, CV_8U, 1.);
    if (!show.empty()) {
      sensor_msgs::ImagePtr disp_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", show).toImageMsg();
      disp_pub.publish(disp_msg);
      publishPointCloud();
      //imshow("view_disp", show);
    }
    /*
    if (cv::waitKey(30) > 0) {
      printf("Saving scene!\n");
      char leftfile[200], rightfile[200], dispfile[200];
      sprintf(leftfile, "left%d.png", save_count);
      sprintf(rightfile, "right%d.png", save_count);
      sprintf(dispfile, "disp%d.png", save_count);
      cv::imwrite(leftfile, leftim_res);
      cv::imwrite(rightfile, rightim_res);
      cv::imwrite(dispfile, show);
      save_count++;
    }*/
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

void imageCallbackRight(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
    //Mat tmp = cv_bridge::toCvShare(msg, "mono8")->image;
    Mat tmp = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_UNCHANGED);
    resize(tmp, img2, rawimsize);
    cv::remap(img2, rightim, rmapx, rmapy, cv::INTER_LINEAR);
    //rightim_res = rightim(Rect(0, 0, 640, 360));
    resize(rightim, rightim_res, Size(270, 180));
    /*
    cv::imshow("view_right", rightim_res);
    if (cv::waitKey(30) > 0) {
      printf("Saving scene!\n");
      char leftfile[200], rightfile[200], dispfile[200];
      sprintf(leftfile, "left%d.png", save_count);
      sprintf(rightfile, "right%d.png", save_count);
      sprintf(dispfile, "disp%d.png", save_count);
      cv::imwrite(leftfile, leftim_res);
      cv::imwrite(rightfile, rightim_res);
      cv::imwrite(dispfile, show);
      save_count++;
    }*/
  }
  catch (cv_bridge::Exception& e)
  {
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener_left");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  disp_pub = it.advertise("/webcam_left/depth_map", 10);
  pcl_publisher = nh.advertise<sensor_msgs::PointCloud>("/webcam_left/point_cloud", 10);
  
  cv::namedWindow("view_left");
  cv::namedWindow("view_right");
  cv::namedWindow("view_disp");

  cv::FileStorage fs1("/home/sourish/catkin_ws/src/jackal_nav/src/calibration/stereo_calib.yml", cv::FileStorage::READ);
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

  cv::startWindowThread();
  ros::Subscriber subl = nh.subscribe("/webcam_left/image_raw/compressed", 10, imageCallbackLeft);
  ros::Subscriber subr = nh.subscribe("/webcam_right/image_raw/compressed", 10, imageCallbackRight);
  
  ros::spin();
  cv::destroyWindow("view_left");
  cv::destroyWindow("view_right");
  cv::destroyWindow("view_disp");
}