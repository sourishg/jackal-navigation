## Obstacle Avoidance using stereo RGB cameras

### Dependencies

- ROS
- OpenCV

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
rosrun jackal_nav point_cloud -c=path/to/calib/file [options]
```

options:
- `-h=height`, specify height of the left and right image from the top, so that only a partial disparity map can be computed
- `-g`, generates point cloud before obstacle scan
- `-l`, log time taken for each step
- `-d=path/to/dmap/time/file`, specify the file where time taken by disparity map is stored for each frame
- `-p=path/to/pcl/time/file`, specify the file where time taken to generate a point cloud is stored for each frame
- `-s=path/to/scan/time/file`, specify the file where time taken to scan for obstacles is stored for each frame

Subscribes to two camera topics: 

- `/webcam/left/image_raw/compressed`
- `/webcam/right/image_raw/compressed`

Publishes three topics: 

- `/webcam/left/depth_map`
- `/webcam/left/point_cloud`
- `/webcam/left/obstacle_scan`

Now the run the `navigate` node for safe navigation. 

```bash
rosrun jackal_nav navigate [options]
```

options:
- `-f=max_forward_vel`, specify maximum forward velocity
- `-l=laser_scan_threshold`, specify a threshold for the number of laser points in front of the robot which determines whether there is an obstacle or not

### Drive modes

Use the following combos on the DualShock controller to switch between modes

- `R1 + R2`: Use left stick to drive. The jackal stops in front of obstacles.
- `Hold X`: Let the Jackal move in the direction it's facing, avoiding obstacles in its way.
- `Hold triangle`: Auto waypoint based navigation

### Disparity Maps

Dense disparity maps are generated using [libelas](http://www.cvlibs.net/software/libelas/) which is a great open source library for realtime generation of disparity maps.
