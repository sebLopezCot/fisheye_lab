#include "load_calibration.h"
#include <iostream>

int main() {
    try {
        // Test camera to pose calibration
        std::cout << "Loading camera to pose calibration..." << std::endl;
        auto cam_to_pose = kitti360::loadCalibrationCameraToPose("calib_cam_to_pose.txt");
        
        for (const auto& [camera, transform] : cam_to_pose) {
            std::cout << camera << ":" << std::endl;
            std::cout << transform << std::endl << std::endl;
        }
        
        // Test camera to velodyne calibration
        std::cout << "Loading camera to velodyne calibration..." << std::endl;
        cv::Mat cam_to_velo = kitti360::loadCalibrationRigid("calib_cam_to_velo.txt");
        std::cout << "calib_cam_to_velo:" << std::endl;
        std::cout << cam_to_velo << std::endl << std::endl;
        
        // Test SICK to velodyne calibration
        std::cout << "Loading SICK to velodyne calibration..." << std::endl;
        cv::Mat sick_to_velo = kitti360::loadCalibrationRigid("calib_sick_to_velo.txt");
        std::cout << "calib_sick_to_velo:" << std::endl;
        std::cout << sick_to_velo << std::endl << std::endl;
        
        // Test perspective intrinsics
        std::cout << "Loading perspective intrinsics..." << std::endl;
        auto perspective = kitti360::loadPerspectiveIntrinsic("perspective.txt");
        
        for (const auto& [param, matrix] : perspective) {
            std::cout << param << ":" << std::endl;
            std::cout << matrix << std::endl << std::endl;
        }
        
        // Test fisheye parameters
        std::cout << "Loading fisheye parameters..." << std::endl;
        auto fisheye02 = kitti360::loadFisheyeParams("image_02.yaml");
        std::cout << "Camera: " << fisheye02.camera_name << std::endl;
        std::cout << "Image size: " << fisheye02.image_width << "x" << fisheye02.image_height << std::endl;
        std::cout << "Mirror parameter (xi): " << fisheye02.xi << std::endl;
        std::cout << "Distortion (k1, k2, p1, p2): " << fisheye02.distortion << std::endl;
        std::cout << "Projection (gamma1, gamma2, u0, v0): " << fisheye02.projection << std::endl << std::endl;
        
        auto fisheye03 = kitti360::loadFisheyeParams("image_03.yaml");
        std::cout << "Camera: " << fisheye03.camera_name << std::endl;
        std::cout << "Image size: " << fisheye03.image_width << "x" << fisheye03.image_height << std::endl;
        std::cout << "Mirror parameter (xi): " << fisheye03.xi << std::endl;
        std::cout << "Distortion (k1, k2, p1, p2): " << fisheye03.distortion << std::endl;
        std::cout << "Projection (gamma1, gamma2, u0, v0): " << fisheye03.projection << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}