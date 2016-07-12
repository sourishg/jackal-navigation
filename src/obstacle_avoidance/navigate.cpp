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
bool obstacle_course_trigger = 0;

const int INF = 1e9;

void visualizeLaserPoints() {
  // visualize laser points from obstacle scan as Marker points
  visualization_msgs::Marker points;
  points.header.frame_id = "jackal";
  points.header.stamp = ros::Time::now();
  points.ns = "jackal_navigation";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.02;
  points.color.b = 1.0;
  points.color.a = 1.0;
  for (int i = 0; i < laserPoints.size(); i++) {
    geometry_msgs::Point p;
    p.x = laserPoints[i].x;
    p.y = laserPoints[i].y;
    p.z = 0;
    points.points.push_back(p);
  }
  marker_pub.publish(points);
}

int checkObstacle() {
  int count = 0;
  int laser_pt_thresh = 7;
  int isObstacle = 0;
  // declare clearances in front and side of the robot
  double clear_front = 1.7;
  double clear_side = 0.3;
  for (int i = 0; i < laserPoints.size(); i++) {    
    // check if laser points lie in safe region
    if (laserPoints[i].x > 0. && laserPoints[i].x < clear_front
        && laserPoints[i].y > -clear_side && laserPoints[i].y < clear_side) {
      count++;
    }
  }
  // spatial filter
  if (count > laser_pt_thresh) {
    isObstacle = 1;
  }
  return isObstacle;
}

int chooseDirection() {
  double left_score = 0., right_score = 0.;
  for (int i = 0; i < laserPoints.size(); i++) {
    double dist = sqrt(laserPoints[i].x*laserPoints[i].x + laserPoints[i].y*laserPoints[i].y);
    if (laserPoints[i].y < 0) {
      right_score += dist;
    } else {
      left_score += dist;
    } 
  }
  if (left_score > right_score)
    return 1;
  return 0;
}

double getSafeVel(double trans_accel) {
  // get maximum allowable velocity in front of obstacle
  double minDist = INF;
  for (int i = 0; i < laserScan[i]; i++) {
    minDist = min(minDist, laserScan[i]);
  }
  return sqrt(2 * trans_accel * minDist);
}

void safeNavigate(const sensor_msgs::JoyConstPtr& msg) {
  int trigger = msg->buttons[9];
  // if RT is not triggered, do nothing
  if (!trigger) {
    // forward_vel = rot_vel = 0.;
    return;
  }
  double side = msg->axes[0];
  double front = msg->axes[1];
  double trans_accel = 0.025;
  double rot_accel = 0.05;
  double max_forward_vel = 0.5;
  double max_rot_vel = 1.3;
  double desired_forward_vel = max_forward_vel * front;
  double desired_rot_vel = max_rot_vel * side;
  int dir = checkObstacle();
  if (dir == 1) {
    // stop-in-front-of-obstacle mode
    if (msg->buttons[11]) {
      desired_forward_vel = min(desired_forward_vel, 0.);
    }
    // rotate-in-front-of obstacle mode 
    else {
      desired_rot_vel = 1.3;
      desired_forward_vel = min(desired_forward_vel, getSafeVel(trans_accel));
    }
  }
  // accelerate or decelerate accordingly
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
  geometry_msgs::Twist vel_msg;
  vel_msg.linear.x = forward_vel;
  vel_msg.angular.z = rot_vel;
  vel_pub.publish(vel_msg);
}

void runObstacleCourse(const sensor_msgs::JoyConstPtr& msg) {
  int trigger = msg->buttons[14];
  if (!trigger) {
    // forward_vel = rot_vel = 0.;
    return;
  }
  double trans_accel = 0.025;
  double rot_accel = 0.05;
  double max_forward_vel = 0.5;
  double max_rot_vel = 1.3;
  double desired_forward_vel;
  double desired_rot_vel;
  int obst = checkObstacle();
  if (obst) {
    int dir = chooseDirection();
    if (dir) {
      desired_rot_vel = max_rot_vel * 0.95;
    } else {
      desired_rot_vel = max_rot_vel * 0.95 * (-1);
    }
    desired_forward_vel = max_forward_vel * 0.4;
  } else {
    desired_forward_vel = max_forward_vel * 0.95;
    desired_rot_vel = max_rot_vel * 0.0;
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
  // read laser points from laser scan
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
  ros::Subscriber obst_course_drive = nh.subscribe("/bluetooth_teleop/joy", 10, runObstacleCourse);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualize_laser", 10);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 10);
  ros::spin();
  return 0;
}