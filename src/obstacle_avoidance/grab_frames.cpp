#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include "popt_pp.h"

using namespace cv;
using namespace std;
Mat imgL, imgR, imgL_resized, imgR_resized;
int save_count = 1;

int im_width = 640;
int im_height = 360;

void imageCallbackLeft(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //imgL = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_UNCHANGED);
    imgL = cv_bridge::toCvShare(msg, "bgr8")->image;
    //cv::resize(imgL, imgL_resized, cv::Size(im_width, im_height));
    Mat imdisp;
    resize(imgL, imdisp, Size(640, 360));
    cv::imshow("view_left", imdisp);
    if (cv::waitKey(30) > 0) {
      printf("Saving scene!\n");
      char leftfile[200], rightfile[200];
      sprintf(leftfile, "left%d.jpg", save_count);
      sprintf(rightfile, "right%d.jpg", save_count);
      cv::imwrite(leftfile, imgL);
      cv::imwrite(rightfile, imgR);
      save_count++;
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("%s", e.what());
  }
}

void imageCallbackRight(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //imgR = cv::imdecode(cv::Mat(msg->data), CV_LOAD_IMAGE_UNCHANGED);
    imgR = cv_bridge::toCvShare(msg, "bgr8")->image;
    //cv::resize(imgR, imgR_resized, cv::Size(im_width, im_height));
    Mat imdisp;
    resize(imgR, imdisp, Size(640, 360));
    cv::imshow("view_right", imdisp);
    if (cv::waitKey(30) > 0) {
      printf("Saving scene!\n");
      char leftfile[200], rightfile[200];
      sprintf(leftfile, "left%d.jpg", save_count);
      sprintf(rightfile, "right%d.jpg", save_count);
      cv::imwrite(leftfile, imgL);
      cv::imwrite(rightfile, imgR);
      save_count++;
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("%s", e.what());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener_left");
  ros::NodeHandle nh;

  static struct poptOption options[] = {
    { "width",'w',POPT_ARG_INT,&im_width,0,"Image width","NUM" },
    { "height",'h',POPT_ARG_INT,&im_height,0,"Image height","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}

  cv::namedWindow("view_left");
  cv::namedWindow("view_right");
  cv::startWindowThread();
  imgL = Mat(im_height, im_width, CV_8UC3, Scalar(0,0,0));
  imgR = Mat(im_height, im_width, CV_8UC3, Scalar(0,0,0));
  ros::Subscriber subl = nh.subscribe("/webcam/left/image_raw", 1, imageCallbackLeft);
  ros::Subscriber subr = nh.subscribe("/webcam/right/image_raw", 1, imageCallbackRight);
  cout << "Press any key to save an image pair!" << endl;
  ros::spin();
  cv::destroyWindow("view_left");
  cv::destroyWindow("view_right");
}