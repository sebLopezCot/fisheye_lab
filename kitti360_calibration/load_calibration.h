#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <map>
#include <vector>

namespace kitti360 {

/**
 * @brief Check if file exists and is readable
 * @param filename Path to the file to check
 * @throws std::runtime_error if file doesn't exist
 */
void checkFile(const std::string& filename);

/**
 * @brief Read a variable from calibration file
 * @param file Input file stream
 * @param name Variable name to search for
 * @param rows Number of rows in the matrix
 * @param cols Number of columns in the matrix
 * @return OpenCV matrix containing the data, or empty matrix if not found
 */
cv::Mat readVariable(std::ifstream& file, const std::string& name, int rows, int cols);

/**
 * @brief Load camera to pose transformation matrices
 * @param filename Path to calib_cam_to_pose.txt
 * @return Map of camera names to 4x4 transformation matrices
 */
std::map<std::string, cv::Mat> loadCalibrationCameraToPose(const std::string& filename);

/**
 * @brief Load rigid body transformation matrix
 * @param filename Path to calibration file (e.g., calib_cam_to_velo.txt)
 * @return 4x4 transformation matrix
 */
cv::Mat loadCalibrationRigid(const std::string& filename);

/**
 * @brief Load perspective camera intrinsic parameters
 * @param filename Path to perspective.txt
 * @return Map of parameter names to matrices (P_rect_XX, R_rect_XX)
 */
std::map<std::string, cv::Mat> loadPerspectiveIntrinsic(const std::string& filename);

/**
 * @brief Structure to hold fisheye camera parameters
 */
struct FisheyeParams {
    std::string camera_name;
    int image_width;
    int image_height;
    double xi;  // Mirror parameter
    cv::Vec4d distortion;  // k1, k2, p1, p2
    cv::Vec4d projection;  // gamma1, gamma2, u0, v0
};

/**
 * @brief Load fisheye camera parameters from YAML file
 * @param filename Path to image_XX.yaml file
 * @return FisheyeParams structure
 */
FisheyeParams loadFisheyeParams(const std::string& filename);

} // namespace kitti360