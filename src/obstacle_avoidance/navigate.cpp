#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;

vector< Point2d > laserPoints;
vector< double > laserScan;
vector< double > laserAngles;

ros::Publisher marker_pub;
ros::Publisher vel_pub;

double forward_vel, rot_vel;

const int INF = 1e9;

void visualizeLaserPoints() {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "map";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "jackal_navigation";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0;
  line_strip.type = visualization_msgs::Marker::POINTS;
  line_strip.scale.x = 0.02;
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;
  for (int i = 0; i < laserPoints.size(); i++) {
    geometry_msgs::Point p;
    p.x = laserPoints[i].x;
    p.y = laserPoints[i].y;
    p.z = 0;
    line_strip.points.push_back(p);
  }
  marker_pub.publish(line_strip);
}

int checkObstacle() {
  int count = 0;
  int laser_pt_thresh = 10;
  int dir = 0;
  double clear_front = 1.0;
  double clear_side = 0.4;
  for (int i = 0; i < laserPoints.size(); i++) {    
    if (laserPoints[i].x > 0. && laserPoints[i].x < clear_front
        && laserPoints[i].y > -clear_side && laserPoints[i].y < clear_side) {
      count++;
    }
  }
  if (count > laser_pt_thresh) {
    int left_count = 0, right_count = 0;
    for (int i = 0; i < laserPoints.size(); i++) {    
      if (laserPoints[i].y > 0)
        left_count++;
      else
        right_count++;
    }
    if (right_count > left_count)
      dir = 2;
    else
      dir = 1;
  }
  return dir;
}

void safeNavigate(const sensor_msgs::JoyConstPtr& msg) {
  int trigger = msg->buttons[9];
  if (!trigger) 
    return;
  double side = msg->axes[0];
  double front = msg->axes[1];
  double max_forward_vel = 0.4;
  double max_rot_vel = 1.4;
  forward_vel = max_forward_vel * front;
  rot_vel = max_rot_vel * side;
  int dir = checkObstacle();
  if (dir == 1) {
    rot_vel = 0.8;
  } else if (dir == 2) {
    rot_vel = -0.8;
  }
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = forward_vel;
  vel_msg.angular.z = rot_vel;
  vel_pub.publish(vel_msg);
}

void laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  unsigned int numPoints = msg->ranges.size();
  Point2d laserScannerLocation(0.0,0.0);
  double minAngle = msg->angle_min;
  double maxAngle = msg->angle_max;
  double angle;

  if (numPoints != laserPoints.size()) {
    laserPoints.resize(numPoints);
    laserScan.resize(numPoints);
    laserAngles.resize(numPoints);
  }

  for (int i = 0; i < msg->ranges.size(); i++) {
    /*
    angle = ((double)(msg->ranges.size() - i)/msg->ranges.size())*(maxAngle-minAngle)+minAngle + 1.5708;
    laserAngles[i] = angle;
    if (msg->ranges[i]<=msg->range_min || msg->ranges[i]>=msg->range_max) {
      laserScan[i] = INF;
      laserPoints[i].x = INF;
      laserPoints[i].y = INF;
    } else {
      laserScan[i] = msg->ranges[i];
      laserPoints[i].x = msg->ranges[i] * sin(angle) + laserScannerLocation.x;
      laserPoints[i].y = msg->ranges[i] * cos(angle) + laserScannerLocation.y;
    }
    */
    angle = (double)i * (maxAngle - minAngle) / (double)numPoints + minAngle;
    laserScan[i] = msg->ranges[i];
    laserPoints[i].x = msg->ranges[i] * cos(angle) + laserScannerLocation.x;
    laserPoints[i].y = msg->ranges[i] * sin(angle) + laserScannerLocation.y;
  }
  visualizeLaserPoints();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jackal_navigation");
  ros::NodeHandle nh;
  ros::Subscriber sub_laser_scan = nh.subscribe("/webcam_left/obstacle_scan", 10, laserScanCallback);
  //ros::Subscriber sub_safe_drive = nh.subscribe("/bluetooth_teleop/joy", 10, safeNavigate);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualize_laser", 10);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 10);
  ros::spin();
  return 0;
}