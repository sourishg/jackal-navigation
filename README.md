## Setup guide for obstacle avoidance on the Jackal

### Step 1: Clone the source

Setup a static IP connection between your computer and the Jackal. If you're doing this over ethernet, set your IP to `192.168.1.X` where `X` can be anything except `11`. The Jackal's IP address is `192.168.11`. Then do the following:

```bash
$ ssh administrator@192.168.1.11
```

The password is `clearpath`.

```bash
~$ cd ~/catkin_ws
~/catkin_ws$ git clone https://github.com/sourishg/jackal-navigation
~/catkin_ws$ catkin_make
```

Clone and make the repository in the Intel NUC. The NUC does all the processing for obstacle avoidance. You need to setup remote ROS between the NUC and the Jackal's computer. Make sure you set up a two-way connection (check this by pinging one computer from another). `rostopic echo` to see if you can receive the topics published by the Jackal on the NUC (or your computer).

### Step 2: Start the cameras

SSH into the Jackal to start the cameras.

Install UVC camera driver for the webcams:

```bash
$ sudo apt-get install ros-<distro>-uvc-camera
```

#### Logitech C920 webcams

```bash
$ sudo chmod a+rw /dev/video*
$ roslaunch jackal_nav stereo.launch
```

If this fails, just repeat the above steps till it works! The camera topics are:

- `/webcam/left/image_raw/compressed`
- `/webcam/right/image_raw/compressed`

#### PointGrey IMX249

The launch files for the PointGrey cameras are stored in the `pointgrey_camera_driver` package. `roscd` into the package if you want to edit the launch files. Check the ID of the cameras by running

```bash
$ rosrun pointgrey_camera_driver list_cameras
```

If you have the launch files ready, then start the cameras by running

```bash
$ roslaunch pointgrey_camera_driver camera_left.launch
```

```bash
$ roslaunch pointgrey_camera_driver camera_right.launch
```

The camera topics are:

- `/camera_left/image_color/compressed`
- `/camera_right/image_color/compressed`

Make sure the cameras are actually grabbing frames by running `rostopic echo` on these topics.

### Step 3: Camera Calibration

Grab images of the checkerboard to calibrate the cameras on the Jackal. Run the following node:

```bash
$ rosrun jackal_nav grab_frames [options]
```

options:

- `-w=width`, specify image width
- `-h=height`, specify image height

Once you have the calibration images saved, use [this tool](https://github.com/sourishg/stereo-calibration) to calibrate for the intrinsics and the extrinsics. An example calibration file is saved in `calibration/amrl_jackal_webcam_stereo.yml`. The `XR` and `XT` matrices in the calibration file are the transformation matrices from the camera frame to the robot frame. 

Initially after stereo calibration (using the tool mentioned above) you will not have the `XR` and `XT` matrices in your calibration file. Just manually add the two matrices and set them to the identity and zero matrices respectively. Also, you only need the following matrices in your calibration file: `K1`, `K2`, `D1`, `D2`, `R`, `T`, `XR`, and `XT`.

Now you need to perform the extrinsic calibration between the rover reference frame and the left camera reference frame to detect the ground plane. To do this, run the `point_cloud` binary with the `-g` and `-m` flag (described below). Then run

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

Tweak the rotation and translation matrix parameters and see the observed point cloud in an `rviz` window. Visually align the point cloud so that the ground plane aligns with rviz's ground plane. After this is done, copy the transformation matrices that are printed on the terminal running the `point_cloud` binary to the calibration file.

### Step 4: Generate disparity map, point cloud, and obstacle scan

Once the camera topics are being published, generate a point cloud and an obstacle scan using this command.

```bash
$ rosrun jackal_nav point_cloud -c=path/to/calib/file [options]
```

options:
- `-h=height`, specify height of the left and right image from the top, so that only a partial disparity map can be computed
- `-g`, generates point cloud before obstacle scan
- `-l`, log time taken for each step
- `-d=path/to/dmap/time/file`, specify the file where time taken by disparity map is stored for each frame
- `-p=path/to/pcl/time/file`, specify the file where time taken to generate a point cloud is stored for each frame
- `-s=path/to/scan/time/file`, specify the file where time taken to scan for obstacles is stored for each frame
- `-m`, flag to perform extrinsic calibration between the rover reference frame and the left camera reference frame.

### Step 5: Safe navigation

Now the run the `navigate` node for safe navigation.

```bash
$ rosrun jackal_nav navigate [options]
```

options:
- `-f=max_forward_vel`, specify maximum forward velocity
- `-l=laser_scan_threshold`, specify a threshold for the number of laser points in front of the robot which determines whether there is an obstacle or not
- `-c=forward_clearance`, the forward clearance for the robot (*e.g.* 1m forward clearance will make the robot stop if there's an obstacle within 1m)

### Drive modes

Use the following combos on the DualShock controller to switch between modes

- `R1 + R2`: Use left stick to drive. The jackal stops in front of obstacles.
- `Hold X`: Let the Jackal move in the direction it's facing, avoiding obstacles in its way. Adjust speed using the left stick.
