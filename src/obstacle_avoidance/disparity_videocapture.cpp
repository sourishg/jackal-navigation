#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include "elas.h"
#include "image.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ros::Publisher left_pub = nh.advertise<sensor_msgs::Image>("/webcam/left/image_color", 1);
  ros::Publisher right_pub = nh.advertise<sensor_msgs::Image>("/webcam/right/image_color", 1);
  ros::Publisher disp_pub = nh.advertise<sensor_msgs::Image>("/webcam/left/depth_map", 1);
  /*
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  */
  cv::Mat R1, R2, P1, P2, Q;
  cv::Mat K1, K2, R;
  cv::Vec3d T;
  cv::Mat D1, D2;
  Mat img1, img2;
  VideoCapture cap1(0);
  VideoCapture cap2(1);
  cv::FileStorage fs1("/home/sourish/catkin_ws/src/test_pkg/src/calibration/mystereocalib.yml", cv::FileStorage::READ);
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

  cv::Mat lmapx, lmapy, rmapx, rmapy;
  cv::Mat leftim, rightim;
  Mat img_res1, img_res2;
  
  int im_width = 480;
  int im_height = 270;

  cap1 >> img_res1;
  cap2 >> img_res2;
  
  Size rawimsize = Size(im_width, im_height);
  
  resize(img_res1, img1, rawimsize);
  resize(img_res2, img2, rawimsize);
  
  cv::initUndistortRectifyMap(K1, D1, R1, P1, img1.size(), CV_32F, lmapx, lmapy);
  cv::initUndistortRectifyMap(K2, D2, R2, P2, img2.size(), CV_32F, rmapx, rmapy);
  cv::remap(img1, leftim, lmapx, lmapy, cv::INTER_LINEAR);
  cv::remap(img2, rightim, rmapx, rmapy, cv::INTER_LINEAR);
  
  Mat leftdisp, rightdisp;
  Mat l,r;
  l = Mat(rawimsize, CV_8UC1, Scalar(0));
  r = Mat(rawimsize, CV_8UC1, Scalar(0));
  if(leftim.channels()==3){cvtColor(leftim,l,CV_BGR2GRAY);}
  else l=leftim;
  if(rightim.channels()==3)cvtColor(rightim,r,CV_BGR2GRAY);
  else r=rightim;
  /*
  Mat tmp;
  tmp = l(cv::Rect(10,10,100,100));
  l = tmp.clone();
  tmp = r(cv::Rect(10,10,100,100));
  r = tmp.clone();
  */
  int bd = 0;
  Mat lb,rb;
  //cv::copyMakeBorder(l,lb,0,0,bd,bd,cv::BORDER_REPLICATE);
  //cv::copyMakeBorder(r,rb,0,0,bd,bd,cv::BORDER_REPLICATE);
  lb = l(cv::Rect(0,0,im_width,im_height));
  rb = r(cv::Rect(0,0,im_width,im_height));
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
  leftdpf.copyTo(leftdisp);
  //disp.convertTo(leftdisp,CV_16S,1);
  //Mat(rightdpf(cv::Rect(bd,0,leftim.cols,leftim.rows))).copyTo(disp);
  //rightdpf.copyTo(disp);
  //disp.convertTo(rightdisp,CV_16S,16);
  
  int max_disp = -1;
  for (int i = 0; i < im_width; i++) {
    for (int j = 0; j < im_height; j++) {
      if (leftdisp.at<uchar>(j,i) > max_disp) max_disp = leftdisp.at<uchar>(j,i);
    }
  }
  for (int i = 0; i < im_width; i++) {
    for (int j = 0; j < im_height; j++) {
      leftdisp.at<uchar>(j,i) = (int)max(255.0*(float)leftdisp.at<uchar>(j,i)/max_disp,0.0);
    }
  }
  
  Mat show;
  leftdisp.convertTo(show, CV_8U, 1.);
  waitKey(10);

  sensor_msgs::ImagePtr disp_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", show).toImageMsg();
  sensor_msgs::ImagePtr left_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftim).toImageMsg();
  sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightim).toImageMsg();

  ros::Rate loop_rate(10);
  while (nh.ok()) {
    disp_pub.publish(disp_msg);
    left_pub.publish(left_msg);
    right_pub.publish(right_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}