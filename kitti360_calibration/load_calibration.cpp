#include "load_calibration.h"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

namespace kitti360 {

void checkFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error(filename + " does not exist!");
    }
    file.close();
}

cv::Mat readVariable(std::ifstream& file, const std::string& name, int rows, int cols) {
    // Reset file to beginning
    file.clear();
    file.seekg(0, std::ios::beg);
    
    std::string line;
    bool found = false;
    
    // Search for variable identifier
    while (std::getline(file, line)) {
        if (line.find(name + ":") == 0) {
            found = true;
            break;
        }
    }
    
    if (!found) {
        return cv::Mat();
    }
    
    // Remove variable name and colon
    size_t colonPos = line.find(':');
    if (colonPos != std::string::npos) {
        line = line.substr(colonPos + 1);
    }
    
    // Parse the values
    std::stringstream ss(line);
    std::vector<double> values;
    double value;
    
    while (ss >> value) {
        values.push_back(value);
    }
    
    if (values.size() != rows * cols) {
        throw std::runtime_error("Expected " + std::to_string(rows * cols) + 
                                " values, got " + std::to_string(values.size()));
    }
    
    // Create OpenCV matrix
    cv::Mat mat(rows, cols, CV_64F);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat.at<double>(i, j) = values[i * cols + j];
        }
    }
    
    return mat;
}

std::map<std::string, cv::Mat> loadCalibrationCameraToPose(const std::string& filename) {
    checkFile(filename);
    
    std::ifstream file(filename);
    std::map<std::string, cv::Mat> transforms;
    
    std::vector<std::string> cameras = {"image_00", "image_01", "image_02", "image_03"};
    
    for (const auto& camera : cameras) {
        cv::Mat transform3x4 = readVariable(file, camera, 3, 4);
        if (!transform3x4.empty()) {
            // Create 4x4 homogeneous transformation matrix
            cv::Mat transform4x4 = cv::Mat::eye(4, 4, CV_64F);
            transform3x4.copyTo(transform4x4(cv::Rect(0, 0, 4, 3)));
            transforms[camera] = transform4x4;
        }
    }
    
    file.close();
    return transforms;
}

cv::Mat loadCalibrationRigid(const std::string& filename) {
    checkFile(filename);
    
    // Read the 12 values from the file
    std::ifstream file(filename);
    std::vector<double> values;
    double value;
    
    while (file >> value) {
        values.push_back(value);
    }
    file.close();
    
    if (values.size() != 12) {
        throw std::runtime_error("Expected 12 values for rigid transformation, got " + 
                                std::to_string(values.size()));
    }
    
    // Create 4x4 homogeneous transformation matrix
    cv::Mat transform = cv::Mat::eye(4, 4, CV_64F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            transform.at<double>(i, j) = values[i * 4 + j];
        }
    }
    
    return transform;
}

std::map<std::string, cv::Mat> loadPerspectiveIntrinsic(const std::string& filename) {
    checkFile(filename);
    
    std::ifstream file(filename);
    std::map<std::string, cv::Mat> intrinsics;
    
    std::vector<std::string> params = {"P_rect_00", "R_rect_00", "P_rect_01", "R_rect_01"};
    
    for (const auto& param : params) {
        cv::Mat mat;
        if (param.find("P_rect") == 0) {
            // Projection matrix is 3x4
            mat = readVariable(file, param, 3, 4);
            if (!mat.empty()) {
                // Convert to 4x4 homogeneous matrix
                cv::Mat mat4x4 = cv::Mat::eye(4, 4, CV_64F);
                mat.copyTo(mat4x4(cv::Rect(0, 0, 4, 3)));
                intrinsics[param] = mat4x4;
            }
        } else {
            // Rectification matrix is 3x3
            mat = readVariable(file, param, 3, 3);
            if (!mat.empty()) {
                intrinsics[param] = mat;
            }
        }
    }
    
    file.close();
    return intrinsics;
}

FisheyeParams loadFisheyeParams(const std::string& filename) {
    checkFile(filename);
    
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Cannot open YAML file: " + filename);
    }
    
    FisheyeParams params;
    
    fs["camera_name"] >> params.camera_name;
    fs["image_width"] >> params.image_width;
    fs["image_height"] >> params.image_height;
    
    // Read mirror parameter
    fs["mirror_parameters"]["xi"] >> params.xi;
    
    // Read distortion parameters
    double k1, k2, p1, p2;
    fs["distortion_parameters"]["k1"] >> k1;
    fs["distortion_parameters"]["k2"] >> k2;
    fs["distortion_parameters"]["p1"] >> p1;
    fs["distortion_parameters"]["p2"] >> p2;
    params.distortion = cv::Vec4d(k1, k2, p1, p2);
    
    // Read projection parameters
    double gamma1, gamma2, u0, v0;
    fs["projection_parameters"]["gamma1"] >> gamma1;
    fs["projection_parameters"]["gamma2"] >> gamma2;
    fs["projection_parameters"]["u0"] >> u0;
    fs["projection_parameters"]["v0"] >> v0;
    params.projection = cv::Vec4d(gamma1, gamma2, u0, v0);
    
    fs.release();
    return params;
}

} // namespace kitti360