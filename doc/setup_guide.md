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

Clone and make the repository in the Intel NUC too. The NUC does all the processing for the obstacle avoidance. You also need to setup remote ROS between the NUC and the Jackal's computer. There is a wiki on the Cobot project page in redmine on how to do this.

**Note:** Switch to the `kowa-lens` branch on the repository if you're using the KOWA lenses.

### Step 2: Start the cameras

SSH into the Jackal to start the cameras.

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

Once you have the calibration images saved, use [this tool](https://github.com/sourishg/stereo-calibration) to calibrate for the intrinsics and the extrinsics. The calibration file is saved as `src/calibration/stereo_calib.yml`. The `XR` and `XT` matrices in the calibration file are the transformation matrices from the camera frame to the robot frame. You can decompose `XR` into it's Euler angles - see this [link](http://nghiaho.com/?page_id=846).

Make sure you change your `im_width` and `im_height` values based on your calibration resolution.

### Step 4: Generate disparity map, point cloud, and obstacle scan

Once the camera topics are being published, generate a point cloud and an obstacle scan using this command.

```bash
$ rosrun jackal_nav point_cloud -c=path/to/calib/file [options]
```

options:
- `-h=height`, specify height of the left and right image from the top, so that only a partial disparity map can be computed
- `-g`, generates point cloud before obstacle scan
- `-l`, log time taken for each step
- `-l -d=path/to/dmap/time/file`, specify the file where time taken by disparity map is stored for each frame
- `-l -p=path/to/pcl/time/file`, specify the file where time taken to generate a point cloud is stored for each frame
- `-l -s=path/to/scan/time/file`, specify the file where time taken to scan for obstacles is stored for each frame

### Step 5: Safe navigation

Now the run the `navigate` node for safe navigation.

```bash
$ rosrun jackal_nav navigate [options]
```

options:
- `-f=max_forward_vel`, specify maximum forward velocity
- `-l=laser_scan_threshold`, specify a threshold for the number of laser points in front of the robot which determines whether there is an obstacle or not

### Drive modes

Use the following combos on the DualShock controller to switch between modes

- `R1 + R2`: Use left stick to drive. The jackal stops in front of obstacles.
- `Hold X`: Let the Jackal move in the direction it's facing, avoiding obstacles in its way. Adjust speed using the left stick.
- `Hold triangle`: Auto waypoint based navigation
