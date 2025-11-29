# KITTI-360 Fisheye Camera Calibration

This directory contains camera calibration data and utilities for the KITTI-360 dataset's fisheye camera system. The calibration includes transformations between different coordinate frames and intrinsic parameters for both fisheye and perspective cameras.

## Files Overview

### Transform Matrices

#### `calib_cam_to_pose.txt`
**Purpose**: Transforms from camera coordinate frame to vehicle pose coordinate frame
- Contains 4x4 transformation matrices for cameras `image_00`, `image_01`, `image_02`, `image_03`
- Maps 3D points from camera coordinates to the vehicle's global pose frame
- **Usage**: `T_pose_camera = calib_cam_to_pose[camera_id]`

#### `calib_cam_to_velo.txt`
**Purpose**: Transforms from camera coordinate frame to Velodyne LiDAR coordinate frame
- Single 4x4 rigid transformation matrix
- Enables fusion of camera and LiDAR data by mapping camera points to LiDAR coordinates
- **Usage**: `T_velo_cam = calib_cam_to_velo`

#### `calib_sick_to_velo.txt`
**Purpose**: Transforms from SICK LiDAR coordinate frame to Velodyne LiDAR coordinate frame
- Single 4x4 rigid transformation matrix
- Aligns measurements from different LiDAR sensors in the same coordinate system
- **Usage**: `T_velo_sick = calib_sick_to_velo`

### Intrinsic Parameters

#### `perspective.txt`
**Purpose**: Intrinsic calibration for perspective (pinhole) cameras
- Contains rectification and projection matrices for stereo cameras `00` and `01`
- **Parameters**:
  - `K_XX`: 3x3 intrinsic camera matrix
  - `D_XX`: Distortion coefficients
  - `R_XX`: Rectification rotation matrix
  - `P_rect_XX`: Rectified projection matrix
  - `R_rect_XX`: Rectification matrix
- **Usage**: For undistorting and projecting points in perspective cameras

#### `image_02.yaml` / `image_03.yaml`
**Purpose**: Intrinsic calibration for fisheye cameras using MEI (Multiperspective) model
- **Parameters**:
  - `xi`: Mirror parameter for omnidirectional projection
  - `k1`, `k2`, `p1`, `p2`: Distortion coefficients
  - `gamma1`, `gamma2`: Focal length parameters
  - `u0`, `v0`: Principal point coordinates
- **Usage**: For projecting 3D points to fisheye image coordinates and vice versa

## Coordinate Frame Relationships

```
Vehicle Pose ←--[calib_cam_to_pose.txt]-- Camera Frames
     ↕
Velodyne LiDAR ←--[calib_cam_to_velo.txt]-- Camera Frames
     ↕
SICK LiDAR ←--[calib_sick_to_velo.txt]-- Velodyne LiDAR
```

## Usage

The `load_calibration.py` script provides utility functions to load these calibration parameters:

- `loadCalibrationCameraToPose()`: Loads camera-to-pose transforms
- `loadCalibrationRigid()`: Loads rigid body transforms (cam-to-velo, sick-to-velo)
- `loadPerspectiveIntrinsic()`: Loads perspective camera intrinsics

### Example
```python
# Load camera to pose transforms
Tr_cam_to_pose = loadCalibrationCameraToPose('calib_cam_to_pose.txt')
T_pose_cam02 = Tr_cam_to_pose['image_02']

# Load camera to velodyne transform
T_velo_cam = loadCalibrationRigid('calib_cam_to_velo.txt')

# Transform point from camera to velodyne coordinates
point_velo = T_velo_cam @ point_cam_homogeneous
```

## Transform Applications

1. **Multi-sensor fusion**: Align camera, LiDAR, and pose data in common coordinate frames
2. **3D reconstruction**: Project camera features into 3D space using pose information
3. **Sensor calibration**: Validate and refine sensor alignments
4. **Data annotation**: Transfer labels between different sensor modalities