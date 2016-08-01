## Stereo Calibration

### Intrinsics

The two cameras should be calibrated separately for their intrinsics first. The intrinsic parameters include, the camera matrices, distortion coefficients etc. This can be achieved by the checkerboard calibration method.

### Extrinsics

After the intrinsics are calibrated, the extrinsics should be calibrated. The extrinsics include the rotation and translation matrices from one camera to the other, and also the fundamental matrix which relates two corresponding points in the left and right image.

### Rectification Transforms

Additionally one could calculate the rectification transforms so that after image rectification, the corresponding epipolar lines become parallel scan lines. Finding correspondences along scan lines are computationally quicker, and the implementation becomes easier.

### Tools for calibration

Stereo calibration can be done by either manually matching correspondences in the left and right image or by a simpler method -- the checkerboard calibration method. The following two tools can be used to calibrate using the latter method.

- Pinhole model lens: [https://github.com/sourishg/stereo-calibration](https://github.com/sourishg/stereo-calibration)
- Fisheye lens: [https://github.com/sourishg/fisheye-stereo-calibration](https://github.com/sourishg/fisheye-stereo-calibration)

## Camera frame to Robot frame transformations

3D reconstruction from disparity maps give the resulting point cloud in the camera frame. It is essential to transform the point cloud to robot coordinate frame. This would help to track the ground plane which is key for obstacle avoidance.

The transformation basically includes a rotation and a translation matrix which can be computed using SVD given enough corresponding points. This link has a good tutorial and an octave snippet to achieve the same: [http://nghiaho.com/?page_id=671](http://nghiaho.com/?page_id=671)
