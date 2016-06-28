#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace cv;

vector< Point2d > laserPoints;
vector< double > laserScan;
vector< double > laserAngles;

ros::Publisher marker_pub;

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
  line_strip.scale.x = 0.03;
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
int obs = 0;
void checkObstacle() {
  int count = 0;
  for (int i = 0; i < laserPoints.size(); i++) {    
    if (laserPoints[i].x > 0.23 && laserPoints[i].x < 1.23
        && laserPoints[i].y > -0.2 && laserPoints[i].y < 0.2) {
      count++;
    }
  }
  if (count > 10) {
    cout << "Obstacle " << obs << endl;
    obs++;
  }
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
  }
  visualizeLaserPoints();
  checkObstacle();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "jackal_navigation");
  ros::NodeHandle nh;
  ros::Subscriber sub_laser_scan = nh.subscribe("/webcam_left/obstacle_scan", 10, laserScanCallback);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualize_laser", 10);
  ros::spin();
  return 0;
}