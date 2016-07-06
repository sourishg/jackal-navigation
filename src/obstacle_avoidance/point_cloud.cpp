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
#include <visualization_msgs/Marker.h>
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
ros::Publisher obstacle_scan_publisher;
ros::Publisher marker_pub;

int save_count = 1;

Size rawimsize;
int im_width = 270;
int im_height = 180;
int crop_offset_x = 0;
int crop_offset_y = 40;
int crop_im_width = 270;
int crop_im_height = 140;
const int INF = 1e9;

const double gp_height_thresh = 0.06;
const double gp_angle_thresh = 4. * 3.1415 / 180.;
const double gp_dist_thresh = 0.8;

bool inImg(int x, int y) {
  if (x >= 0 && x < show.cols && y >= 0 && y < show.rows)
    return true;
}

/*
vector< Point3d > getfilteredPointCloud() {
  double error_thresh = 0.4;
  vector< Point3d > filtered_pts;
  filtered_pts.resize(im_width * im_height);
  for (int i = 0; i < im_width * im_height; i++) {
    deque< vector< Point3d > >::iterator it;
    vector< Point3d > check_pts;
    for (it = pcls.begin(); it != pcls.end(); it++) {
      check_pts.push_back((*it)[i]);
    }
    double sum_error = 0.;
    int n = check_pts.size();
    int count = 0;
    double X = 0, Y = 0, Z = 0;
    for (int j = 0; j < n; j++) {
      if (check_pts[j].x == -1) {
        continue;
      }
      X += check_pts[j].x;
      Y += check_pts[j].y;
      Z += check_pts[j].z;
      count++;
    }
    if (count == 0) {
      filtered_pts[i] = check_pts[n-1];
      continue;
    }
    X /= count;
    Y /= count;
    Z /= count;
    filtered_pts[i] = Point3d(X, Y, Z);
  }
  return filtered_pts;
}
*/

void filterDisparityMap(Mat& disp) {
  int w = 1;
  for (int i = 0; i < disp.cols; i++) {
    for (int j = 0; j < disp.rows; j++) {
      if (disp.at<uchar>(j,i) == 0)
        continue;
      int val = disp.at<uchar>(j,i);
      int count = 0;
      for (int k = -w; k <= w; k++) {
        for (int l = -w; l <= w; l++) {
          if (inImg(i+k,j+l)) {
            if (abs(val - disp.at<uchar>(i+k,j+l)) > 80) {
              count++;
            }
          }
        }
      }
      if (count > (2*w+1)*(2*w+1)/2)
        disp.at<uchar>(j,i) = 0;
    }
  }
}

void visualizeCriticalRegion() {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "jackal_obstacle_avoidance";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = 0.23;
  p.y = 0.2;
  p.z = 0;
  line_strip.points.push_back(p);
  p.x = 1.23;
  p.y = 0.2;
  p.z = 0;
  line_strip.points.push_back(p);
  p.x = 1.23;
  p.y = -0.2;
  p.z = 0;
  line_strip.points.push_back(p);
  p.x = 0.23;
  p.y = -0.2;
  p.z = 0;
  line_strip.points.push_back(p);
  marker_pub.publish(line_strip);
}

void publishObstacleScan(vector< Point3d > points) {
  double fov = 90.;
  int bin_size = 90;
  double min_angle = 400, max_angle = -400;
  double range_min = INF, range_max = -500;
  double scan[bin_size];
  Point3d closest_pt[bin_size];
  sensor_msgs::LaserScan obstacle_scan;
  obstacle_scan.header.frame_id = "map";
  obstacle_scan.header.stamp = ros::Time::now();
  for (int i = 0; i < bin_size; i++) {
    scan[i] = INF;
  }
  for (int i = 0; i < points.size(); i++) {
    if (points[i].x == -1)
      continue;
    if (points[i].x < gp_dist_thresh) {
      if (points[i].z < gp_height_thresh)
        continue;
    } else {
      if (points[i].z < gp_height_thresh + tan(gp_angle_thresh) * (points[i].x - gp_dist_thresh))
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
}

void publishPointCloud(Mat& show) {
  vector< Point3d > points;
  sensor_msgs::PointCloud pc;
  sensor_msgs::ChannelFloat32 ch;
  ch.name = "rgb";
  pc.header.frame_id = "map";
  pc.header.stamp = ros::Time::now();
  for (int i = 0; i < show.cols; i++) {
    for (int j = 0; j < show.rows; j++) {
      int d = show.at<uchar>(j,i);
      if (d < 2) {
        points.push_back(Point3d(-1, -1, -1));
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
      if (point3d_robot.at<double>(2,0) > 0.34 || point3d_robot.at<double>(2,0) < 0.) {
        points.push_back(Point3d(-1, -1, -1));
        continue;
      }
      points.push_back(Point3d(point3d_robot));
    }
  }
  /*
  if (pcls.size() < 5) {
    pcls.push_back(points);
  } else {
    pcls.pop_front();
    pcls.push_back(points);
    points = getfilteredPointCloud();
  }
  */
  int pts_idx = 0;
  for (int i = 0; i <  show.cols; i++) {
    for (int j = 0; j < show.rows; j++) {
      geometry_msgs::Point32 pt;
      pt.x = points[pts_idx].x;
      pt.y = points[pts_idx].y;
      pt.z = points[pts_idx].z;
      pts_idx++;
      if (pt.x == -1)
        continue;
      pc.points.push_back(pt);
      int32_t red, blue, green;
      if (pt.x < gp_dist_thresh) {
        if (pt.z < gp_height_thresh) {
          red = 0;
          blue = 0;
          green = 255;
        } else {
          red = leftim_res.at<Vec3b>(j,i)[2];
          green = leftim_res.at<Vec3b>(j,i)[1];
          blue = leftim_res.at<Vec3b>(j,i)[0];
        }
      } else {
        if (pt.z < gp_height_thresh + tan(gp_angle_thresh) * (pt.x - gp_dist_thresh)) {
          red = 0;
          blue = 0;
          green = 255;
        } else {
          red = leftim_res.at<Vec3b>(j,i)[2];
          green = leftim_res.at<Vec3b>(j,i)[1];
          blue = leftim_res.at<Vec3b>(j,i)[0];
        }
      }
      int32_t rgb = (red << 16 | green << 8 | blue);
      ch.values.push_back(*reinterpret_cast<float*>(&rgb));
    }
  }
  pc.channels.push_back(ch);
  pcl_publisher.publish(pc);
  publishObstacleScan(points);
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
    //resize(leftim, leftim_res, cropped_imsize);
    //cv::imshow("view_left", leftim_res);
    Mat lb,rb;
    //lb = Mat(cropped_imsize, CV_8UC1, Scalar(0));
    //rb = Mat(cropped_imsize, CV_8UC1, Scalar(0));
    if (leftim_res.empty() || rightim_res.empty()) return;
    cvtColor(leftim_res,lb,CV_BGR2GRAY);
    cvtColor(rightim_res,rb,CV_BGR2GRAY);
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
    
    show = Mat(crop_im_height, crop_im_width, CV_8UC1, Scalar(0));
    leftdpf.convertTo(show, CV_8U, 1.);
    /*
    if (dmaps.size() < 2) {
      dmaps.push_back(show);
    } else {
      dmaps.pop_front();
      dmaps.push_back(show);
      filterDisparityMap();
    }
    */
    if (!show.empty()) {
      //filterDisparityMap(show);
      sensor_msgs::ImagePtr disp_msg;
      //if (dmaps.size() < 2)
      disp_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", show).toImageMsg();
      //else
      //disp_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", dmaps.back()).toImageMsg();
      disp_pub.publish(disp_msg);
      //if (dmaps.size() < 2)
      publishPointCloud(show);
      //else
      //publishPointCloud(dmaps.back());
      //imshow("view_disp", show);
    }
    visualizeCriticalRegion();
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
    if (tmp.empty()) return;
    resize(tmp, img2, rawimsize);
    cv::remap(img2, rightim, rmapx, rmapy, cv::INTER_LINEAR);
    rightim_res = rightim(Rect(crop_offset_x, crop_offset_y, crop_im_width, crop_im_height));
    //resize(rightim, rightim_res, Size(270, 180));
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
  ros::init(argc, argv, "jackal_obstacle_avoidance");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  disp_pub = it.advertise("/webcam_left/depth_map", 10);
  pcl_publisher = nh.advertise<sensor_msgs::PointCloud>("/webcam_left/point_cloud", 10);
  obstacle_scan_publisher = nh.advertise<sensor_msgs::LaserScan>("/webcam_left/obstacle_scan", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //cv::namedWindow("view_left");
  //cv::namedWindow("view_right");
  //cv::namedWindow("view_disp");

  cv::FileStorage fs1(argv[1], cv::FileStorage::READ);
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

  //cv::startWindowThread();
  ros::Subscriber subl = nh.subscribe("/webcam_left/image_raw/compressed", 10, imageCallbackLeft);
  ros::Subscriber subr = nh.subscribe("/webcam_right/image_raw/compressed", 10, imageCallbackRight);

  ros::spin();
  //cv::destroyWindow("view_left");
  //cv::destroyWindow("view_right");
  //cv::destroyWindow("view_disp");
}
