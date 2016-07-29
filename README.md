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
sudo chmod a+rw /dev/video1
roslaunch jackal_nav stereo.launch
```

Calibrate both the intrinsics and the extrinisics of the stereo setup using this [calibration tool](https://github.com/sourishg/stereo-calibration). The calibration file is saved as `src/calibration/stereo_calib.yml`. The `XR` and `XT` matrices in the calibration file are the transformation matrices from the camera frame to the robot frame.

### Safe navigation

Once the camera topics are being published, generate a point cloud and an obstacle scan using this command.

```bash
rosrun jackal_nav point_cloud -h [img_height] -c [path/to/calib/file] -g [generates pcl]
```

For logging time taken for each step in the pipeline use the following command instead

```bash
rosrun jackal_nav point_cloud -h [img_height] -c [path/to/calib/file] -l [logs time] -g [generates pcl] -d [path/to/dmap/time/file] -p [path/to/pcl/time/file] -s [path/to/scan/time/file] 
```

Subscribes to two camera topics: 

- `/webcam/left/image_raw/compressed`
- `/webcam/right/image_raw/compressed`

Publishes three topics: 

- `/webcam/left/depth_map`
- `/webcam/left/point_cloud`
- `/webcam/left/obstacle_scan`

Now the run the `navigate` node for safe navigation. 

```bash
rosrun jackal_nav navigate -f [max_forward_vel] -l [laser_scan_thresh]
```

### Drive modes

Use the following combos on the DualShock controller to switch between modes

- `R1 + R2`: Use left stick to drive. The jackal stops in front of obstacles.
- `Hold X`: Let the Jackal move in the direction it's facing, avoiding obstacles in its way.
- `Hold triangle`: Auto waypoint based navigation

### Disparity Maps

Dense disparity maps are generated using [libelas](http://www.cvlibs.net/software/libelas/) which is a great open source library for realtime generation of disparity maps.
