## Obstacle Avoidance using stereo RGB cameras

This is a ROS package for local obstacle avoidance on the Clearpath Jackal. It performs local 3D reconstruction and then generates an obstacle scan and publishes it as a `sensor_msgs/LaserScan` message. Using the laser scan data, the robot avoids obstacles locally.

### Dependencies

- ROS
- OpenCV
- libelas

### Starting the cameras

Currently using two Logitech C920s as a stereo pair. SSH into the Jackal, and start the webcams using the following commands.

```bash
sudo chmod a+rw /dev/video0
roslaunch jackal_nav webcam_left.launch
```

```bash
sudo chmod a+rw /dev/video1
roslaunch jackal_nav webcam_right.launch
```

You should now get all the necessary camera topics to be processed.

### Disparity Maps

Dense disparity maps are generated using [libelas](http://www.cvlibs.net/software/libelas/) which is a great open source library for realtime generation of disparity maps.
