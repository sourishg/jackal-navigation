## Obstacle Avoidance using stereo RGB cameras

This is a ROS package for local obstacle avoidance on the Clearpath Jackal. It performs local 3D reconstruction and then generates an obstacle scan to and publishes it as a `sensor_msgs/LaserScan` message. Using the laser scan data, the robot plans a path around the obstacle using a standard path planning algorithm - dynamic window for example.

### Dependencies

- ROS Indigo
- OpenCV

### Disparity Maps

Dense disparity maps are generated using [libelas](https://github.com/mjgarcia/cyphy-elas-ros/tree/master/elas/libelas) which is a great open source library for realtime generation of disparity maps.
