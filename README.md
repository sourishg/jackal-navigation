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

### Safe navigation

Once the camera topics are being published, generate a point cloud and an obstacle scan using this command.

```bash
rosrun jackal_nav point_cloud [path/to/calibration/file.yml]
```

It subscribes to two camera topics: `/webcam_left/image_raw/compressed` and `/webcam_right/image_raw/compressed`

It publishes three topics: `/webcam_left/depth_map`, `/webcam_left/point_cloud`, and `/webcam_left/obstacle_scan`

Now the run the `navigate` node for safe navigation. 

```bash
rosrun jackal_nav navigate
```

To drive the Jackal safely hold `R1` + `R2` on the DualShock controller and use the left stick to drive.

### Disparity Maps

Dense disparity maps are generated using [libelas](http://www.cvlibs.net/software/libelas/) which is a great open source library for realtime generation of disparity maps.
