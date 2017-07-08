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
#include <ctime>
#include <fstream>
#include <deque>
#include <geometry_msgs/Pose.h>
#include "popt_pp.h"
#include "jackal_nav/JackalPose.h"

using namespace std;
using namespace cv;

vector< Point2d > laserPoints;
vector< double > laserScan;
vector< double > laserAngles;

ros::Publisher marker_pub;
ros::Publisher vel_pub;

double forward_vel = 0., rot_vel = 0.;
double trans_accel = 0.025; // forward acceleration
double trans_decel = 0.1; // forward deceleration
double rot_accel = 0.05; // rotational acceleration
float max_forward_vel = 0.6; // maximum forward velocity
double max_rot_vel = 1.3; // maximum rotational velocity

// declare clearances in front and side of the robot
double clear_front = 0.24 + 0.8;
double clear_side = 0.3;

// min number of laser points in front of the clearance of the robot
// to detect an obstacle
int laser_pt_thresh = 8;

deque< int > commands;
int last_dir = 0;

const int INF = 1e9;

struct Pose
{
  double x, y, theta;
  double dist(Pose p) {
    return sqrt((x - p.x)*(x - p.x) + (y - p.y)*(y - p.y));
  }
};

struct LineSegment
{
  float x1, y1, x2, y2;
  float getAngle(LineSegment l) {
    float m1 = atan2(y2-y1,x2-x1);
    float m2 = atan2(l.y2-l.y1,l.x2-l.x1);
    return m1 - m2;
  }
  float heading() {
    return atan2(y2-y1,x2-x1);
  }
};

Pose jackal_pos = {0,0,0};
Pose last_jackal_pos = {0,0,0};
Pose current_waypoint;
bool reached_waypoint = false;
deque< Pose > path;
int pose_update_counter = 0;
int rot_frames = 0;

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
  int isObstacle = 0;
  double closestObst = INF;
  for (int i = 0; i < laserPoints.size(); i++) {
    // check if laser points lie in safe region
    double dist = sqrt(laserPoints[i].x*laserPoints[i].x + laserPoints[i].y*laserPoints[i].y);
    closestObst = min(closestObst, dist);
    if (laserPoints[i].x > 0. && laserPoints[i].x < clear_front
        && laserPoints[i].y > -clear_side && laserPoints[i].y < clear_side) {
      count++;
    }
  }
  // spatial filter
  if (count > laser_pt_thresh) {
    isObstacle = 1;
  }
  /* 
  else {
    if (count > 0.5 * (double)laserPoints.size()) {
      isObstacle = 1;
    }
  }
  */
  // if there is a laser point close than 50 cm, the robot should stop
  if (closestObst < 0.5)
    isObstacle = 1;
  // last 20 classifications of obstacles are stored in a deque
  // sort of like a temporal filter
  if (commands.size() < 20) {
    commands.push_back(isObstacle);
  } else {
    commands.pop_front();
    commands.push_back(isObstacle);
  }
  deque< int >::iterator it;
  int one = 0, zero = 0;
  for (it = commands.begin(); it != commands.end(); it++) {
    if ((*it) == 1)
      one++;
    else
      zero++;
  }
  // if there are more than 2 classifications of obstacles in the past 20
  // frames, then it's probably an obstacle
  if (one > 2)
    isObstacle = 1;
  // confidence value of obstacle detection
  double conf = (double)one / (double)(one + zero);
  string stat = (isObstacle == 1) ? "Y" : "N";
  cout << count << ", " << laserPoints.size() << ", " << stat << ", " << closestObst << ", " << conf << endl;
  return isObstacle;
}

int chooseDirection() {
  int left_count = 0, right_count = 0;
  for (int i = 0; i < laserPoints.size(); i++) {
    if (laserPoints[i].x > 0. && laserPoints[i].x < clear_front) {
      // calculate number of laser points in the left and right side
      if (laserPoints[i].y < 0) {
        right_count++;
      } else {
        left_count++;
      }
    }
  }
  if (left_count + right_count < 2)
    return 0;
  // calculate confidence for left turn
  double conf_left = 2.*(double)right_count/(double)(left_count+right_count);
  // calculate confidence for right turn
  double conf_right = 2.*(double)left_count/(double)(left_count+right_count);
  int dir = 0;
  // hysterisis check based on last direction the robot took
  if (conf_left > conf_right) {
    if (last_dir != 1) {
      if (conf_left - conf_right > 0.5) {
        dir = 1;
      } else {
        dir = last_dir;
      }
    } else {
      dir = 1;
    }
  } else {
    if (last_dir != 2) {
      if (conf_right - conf_left > 0.5) {
        dir = 2;
      } else {
        dir = last_dir;
      }
    } else {
      dir = 2;
    }
  }
  return dir;
}

double getSafeVel(double trans_accel) {
  // get maximum allowable velocity in front of obstacle
  double minDist = INF;
  for (int i = 0; i < laserScan[i]; i++) {
    minDist = min(minDist, laserScan[i]);
  }
  return sqrt(2 * trans_accel * minDist);
}

pair< double, double > stopInFrontMode(double side, double front) {
  double desired_forward_vel = max_forward_vel * front;
  double desired_rot_vel = max_rot_vel * side;
  int dir = checkObstacle(); // check for obstacles in front
  if (dir == 1) {
    // if there's an obstacle stop or allow the jackal to move back
    desired_forward_vel = min(desired_forward_vel, 0.);
  }
  return make_pair(desired_forward_vel, desired_rot_vel);
}

pair< double, double > stopInFrontMode() {
  double desired_forward_vel = max_forward_vel * 1.0;
  double desired_rot_vel = 0.0;
  int dir = checkObstacle();
  if (dir == 1) {
    desired_forward_vel = min(desired_forward_vel, 0.);
  }
  return make_pair(desired_forward_vel, desired_rot_vel);
}

pair< double, double > obstacleAvoidMode(double front) {
  double desired_forward_vel;
  double desired_rot_vel;
  int obst = checkObstacle();
  if (obst) {
    int dir = chooseDirection();
    last_dir = dir;
    if (dir == 1) {
      // rotate left
      desired_rot_vel = max_rot_vel * 0.4;
    } else if (dir == 2) {
      // rotate right
      desired_rot_vel = max_rot_vel * 0.4 * (-1);
    } else {
      // dont rotate - no good direction to go
      desired_rot_vel = max_rot_vel * 0.0;
    }
    // stop while rotating
    desired_forward_vel = max_forward_vel * 0.0;
  } else {
    // obstacle free so go forward
    desired_forward_vel = max_forward_vel * max(0.4, front);
    desired_rot_vel = max_rot_vel * 0.0;
    last_dir = 0;
  }
  return make_pair(desired_forward_vel, desired_rot_vel);
}

pair< double, double > goToWayPoint(Pose wayPoint, double front) {
  pair< double, double > ret_vel;
  double dist = wayPoint.dist(jackal_pos);
  if (dist < 3) {
    reached_waypoint = true;
    ret_vel = make_pair(0.,0.);
  } else if (rot_frames != 0) {
    if (rot_frames < 0) {
      cout << "ROTATING LEFT!!!" << endl;
      ret_vel.second = max_rot_vel * 0.5;
      rot_frames++;
    } else {
      cout << "ROTATING RIGHT!!!" << endl;
      ret_vel.second = -max_rot_vel * 0.5;
      rot_frames--;
    }
    ret_vel.first = max_forward_vel * max(0.4, front);
  } else {
    ret_vel.first = max_forward_vel * max(0.4, front);
    ret_vel.second = 0.;
  }
  cout << "Distance to WP: " << dist << endl;
  return ret_vel;
}

pair< double, double > autoNavigateMode(double front) {
  pair< double, double > ret_vel;
  ret_vel = make_pair(0.,0.);
  if (path.size() == 0 && reached_waypoint) {
    cout << "REACHED ALL WAYPOINTS" << endl;
    return ret_vel;
  }
  if (reached_waypoint) {
    cout << "REACHED WAYPOINT! *********************************" << endl;
    current_waypoint = path.front();
    path.pop_front();
    reached_waypoint = false;
  }
  if (!reached_waypoint) {
    ret_vel = goToWayPoint(current_waypoint, front);
  }
  cout << "Current waypoint: " << current_waypoint.x << ", " << current_waypoint.y << endl;
  return ret_vel;
}

void safeNavigate(const sensor_msgs::JoyConstPtr& msg) {
  // read joystick input
  int R2 = msg->buttons[9];
  int R1 = msg->buttons[11];
  int X = msg->buttons[14];
  int O = msg->buttons[13];
  int triangle = msg->buttons[12];
  double side = msg->axes[0];
  double front = msg->axes[1];
  double desired_forward_vel, desired_rot_vel;
  pair< double, double > desired_vel;
  // run the different modes
  if (R1 && R2) {
    desired_vel = stopInFrontMode(side, front);
  } else if (triangle) {
    desired_vel = autoNavigateMode(front); // navigation doesn't work yet
  } else if (X) {
    desired_vel = obstacleAvoidMode(front);
  } else if (O) {
    desired_vel = stopInFrontMode();
  } else {
    return;
  }
  // accelerate or decelerate accordingly
  desired_forward_vel = desired_vel.first;
  desired_rot_vel = desired_vel.second;
  if (desired_forward_vel < forward_vel) {
    forward_vel = max(desired_forward_vel, forward_vel - trans_decel);
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
    laserPoints[i].x = msg->ranges[i] * cos(angle);
    laserPoints[i].y = msg->ranges[i] * sin(angle);
  }
  visualizeLaserPoints();
}

void getCurrentPose(const jackal_nav::JackalPoseConstPtr& msg) {
  jackal_pos.x = msg->x;
  jackal_pos.y = msg->y;
  jackal_pos.theta = msg->theta;
  pose_update_counter++;
  cout << "Current position: " << jackal_pos.x << ", " << jackal_pos.y << endl;
  //cout << "Current: " << jackal_pos.x << ", " << jackal_pos.y << " Prev: " << last_jackal_pos.x << ", " << last_jackal_pos.y << endl;

  LineSegment heading_line = {last_jackal_pos.x,last_jackal_pos.y,jackal_pos.x,jackal_pos.y};
  LineSegment waypoint_line = {jackal_pos.x,jackal_pos.y,current_waypoint.x,current_waypoint.y};
  //cout << "Heading: " << (heading_line.heading() * 180. / 3.14) << endl;
  double ang_diff = heading_line.getAngle(waypoint_line);
  cout << "Ang diff: " << (ang_diff * 180. / 3.14)  << endl;
  cout << "Rot frames: " << rot_frames << endl;

  if (pose_update_counter > 20) {
    if (last_jackal_pos.dist(jackal_pos) > 3) {
      if (abs(ang_diff * 180 / 3.14) > 30) {
        double cmd_rate = 8.;
        rot_frames = ang_diff * cmd_rate / (max_rot_vel * 0.5);
      } else {
        rot_frames = 0;
      }
      last_jackal_pos = jackal_pos;
    }
    pose_update_counter = 0;
  }
}

void read_waypoints(char* filename) {
  ifstream f(filename);
  if (f.is_open()) {
    int n;
    Pose waypoint;
    float x, y;
    waypoint.theta = 0.;
    f >> n;
    for (int i = 0; i < n; i++) {
      f >> x;
      f >> y;
      waypoint.x = x;
      waypoint.y = y;
      cout << "Read (" << waypoint.x << "," << waypoint.y << ")" << endl;
      path.push_back(waypoint);
    }
    current_waypoint = path.front();
    path.pop_front();
    cout << "Read waypoints!" << endl;
  } else {
    cout << "Cannot open file!" << endl;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jackal_navigation");
  ros::NodeHandle nh;

  static struct poptOption options[] = {
    { "max-forward-vel",'f',POPT_ARG_FLOAT,&max_forward_vel,0,"Max forward velocity","NUM" },
    { "laser-thresh",'l',POPT_ARG_INT,&laser_pt_thresh,0,"Threshold for obstacle scan","NUM" },
    { "forward-clearance",'c',POPT_ARG_FLOAT,&clear_front,0,
      "Forward clearance range","NUM" },
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };

  POpt popt(NULL, argc, argv, options, 0);
  int c;
  while((c = popt.getNextOpt()) >= 0) {}
  
  ros::Subscriber sub_laser_scan = nh.subscribe("/webcam/left/obstacle_scan", 1, laserScanCallback);
  ros::Subscriber sub_safe_drive = nh.subscribe("/bluetooth_teleop/joy", 1, safeNavigate);
  ros::Subscriber sub_cur_pose = nh.subscribe("/jackal/gps_estimate", 1, getCurrentPose);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualize_laser", 1);
  vel_pub = nh.advertise<geometry_msgs::Twist>("/jackal_velocity_controller/cmd_vel", 1);
  ros::spin();
  return 0;
}
