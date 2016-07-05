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

double forward_vel = 0., rot_vel = 0.;
int last_dir = 0;

const int INF = 1e9;

void capture_frame_fisheye(const sensor_msgs::JoyConstPtr& msg) {
  int trigger = msg->buttons[11];
  if (trigger) {
    // code for capturing frame
  }
}

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
  double clear_front = 1.7;
  double clear_side = 0.3;
  for (int i = 0; i < laserPoints.size(); i++) {    
    if (laserPoints[i].x > 0. && laserPoints[i].x < clear_front
        && laserPoints[i].y > -clear_side && laserPoints[i].y < clear_side) {
      count++;
    }
  }
  if (count > laser_pt_thresh) {
    /*
    double max_angle = -100, min_angle = 100;
    for (int i = 0; i < laserPoints.size(); i++) {
      double angle = atan2(laserPoints[i].y, laserPoints[i].x);
      if (laserPoints[i].x > clear_front)
        continue;
      max_angle = max(max_angle, angle);
      min_angle = min(min_angle, angle);
    }
    if (max_angle < 0)
      dir = 1;
    else if (min_angle > 0)
      dir = 2;
    else if (abs(max_angle) > abs(min_angle))
      dir = 2;
    else
      dir = 1;
    */
    dir = 1;
  }
  return dir;
}

double getSafeVel(double trans_accel) {
  double minDist = 1000;
  for (int i = 0; i < laserScan[i]; i++) {
    minDist = min(minDist, laserScan[i]);
  }
  return sqrt(2 * trans_accel * minDist);
}

void safeNavigate(const sensor_msgs::JoyConstPtr& msg) {
  int trigger = msg->buttons[9];
  if (!trigger) {
    forward_vel = rot_vel = 0.;
    return;
  }
  double side = msg->axes[0];
  double front = msg->axes[1];
  double trans_accel = 0.025;
  double rot_accel = 0.05;
  double max_forward_vel = 1.0;
  double max_rot_vel = 1.4;
  double desired_forward_vel = max_forward_vel * front;
  double desired_rot_vel = max_rot_vel * side;
  int dir = checkObstacle();
  if (dir == 1) {
    desired_rot_vel = 1.2;
    desired_forward_vel = min(desired_forward_vel, getSafeVel(trans_accel));
  }
  if (desired_forward_vel < forward_vel) {
    forward_vel = max(desired_forward_vel, forward_vel - trans_accel);
  } else {
    forward_vel = min(desired_forward_vel, forward_vel + trans_accel);
  }
  if (desired_rot_vel < rot_vel) {
    rot_vel = max(desired_rot_vel, rot_vel - rot_accel);
  } else {
    rot_vel = min(desired_rot_vel, rot_vel + rot_accel);
  }
  /*
  if (dir == 0) {
    last_dir = 0;
  } else {
    if (last_dir != 0) {
      if (dir != last_dir) {
        dir = last_dir;
      }
    }
  }
  if (dir == 0) cout << "Forward!" << endl;
  else if (dir == 1) cout << "Left!" << endl;
  else cout << "Right!" << endl;
  if (dir == 1) {
    forward_vel = 0.2 * front;
    rot_vel = min(rot_vel + 0.23, max_rot_vel);
  } else if (dir == 2) {
    rot_vel = max(rot_vel - 0.23, -max_rot_vel);
    forward_vel = 0.2 * front;
  }
  last_dir = dir;
  */
  cout << "F: " << forward_vel << " R: " << rot_vel << endl;
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
  ros::Subscriber sub_safe_drive = nh.subscribe("/bluetooth_teleop/joy", 10, safeNavigate);
  ros::Subscriber cap_fisheye = nh.subscribe("/bluetooth_teleop/joy", 10, capture_frame_fisheye);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualize_laser", 10);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 10);
  ros::spin();
  return 0;
}