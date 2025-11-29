#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "kitti360_calibration/load_calibration.h"
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;

class FisheyeUndistorter {
private:
    kitti360::FisheyeParams cameraParams;
    cv::Mat cameraMatrix, distCoeffs;
    cv::Mat mapX, mapY;
    cv::Size outputImageSize;  // Size for the unwrapped output image
    bool calibrationLoaded;
    
public:
    FisheyeUndistorter() : calibrationLoaded(false) {}
    
    bool loadCalibration() {
        try {
            std::cout << "=== LOADING FISHEYE CALIBRATION PARAMETERS ===" << std::endl;
            
            // Load fisheye parameters for left camera (image_02)
            cameraParams = kitti360::loadFisheyeParams("kitti360_calibration/image_02.yaml");
            
            std::cout << "✓ Successfully loaded calibration file: kitti360_calibration/image_02.yaml" << std::endl;
            std::cout << "Camera: " << cameraParams.camera_name << std::endl;
            std::cout << "Image size: " << cameraParams.image_width << "x" << cameraParams.image_height << std::endl;
            std::cout << "Xi parameter (mirror): " << cameraParams.xi << std::endl;
            std::cout << "Distortion parameters:" << std::endl;
            std::cout << "  k1 = " << cameraParams.distortion[0] << std::endl;
            std::cout << "  k2 = " << cameraParams.distortion[1] << std::endl;
            std::cout << "  p1 = " << cameraParams.distortion[2] << std::endl;
            std::cout << "  p2 = " << cameraParams.distortion[3] << std::endl;
            std::cout << "Projection parameters:" << std::endl;
            std::cout << "  gamma1 (fx) = " << cameraParams.projection[0] << std::endl;
            std::cout << "  gamma2 (fy) = " << cameraParams.projection[1] << std::endl;
            std::cout << "  u0 (cx) = " << cameraParams.projection[2] << std::endl;
            std::cout << "  v0 (cy) = " << cameraParams.projection[3] << std::endl;
            
            // Setup camera matrix and distortion coefficients
            setupCameraParameters();
            
            // Create undistortion maps
            createUndistortionMaps();
            
            calibrationLoaded = true;
            std::cout << "✓ Calibration loaded and undistortion maps created successfully!" << std::endl;
            std::cout << "================================================" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ CRITICAL ERROR: Failed to load calibration: " << e.what() << std::endl;
            std::cerr << "✗ Current working directory: " << std::filesystem::current_path() << std::endl;
            std::cerr << "✗ Expected calibration file: kitti360_calibration/image_02.yaml" << std::endl;
            if (!std::filesystem::exists("kitti360_calibration/image_02.yaml")) {
                std::cerr << "✗ Calibration file does NOT exist!" << std::endl;
            } else {
                std::cerr << "✗ Calibration file exists but failed to load - check file format" << std::endl;
            }
            calibrationLoaded = false;
            return false;
        }
    }
    
    void setupCameraParameters() {
        // Setup camera matrix from fisheye parameters
        cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        cameraMatrix.at<double>(0, 0) = cameraParams.projection[0]; // gamma1 (fx)
        cameraMatrix.at<double>(1, 1) = cameraParams.projection[1]; // gamma2 (fy)
        cameraMatrix.at<double>(0, 2) = cameraParams.projection[2]; // u0 (cx)
        cameraMatrix.at<double>(1, 2) = cameraParams.projection[3]; // v0 (cy)
        
        std::cout << "Camera matrix:" << std::endl << cameraMatrix << std::endl;
        
        // For fisheye model, use all 4 distortion parameters
        // Map MEI model parameters to OpenCV fisheye model
        // k1, k2 are radial distortion (same in both models)
        // p1, p2 from MEI can be mapped to k3, k4 in OpenCV fisheye model
        distCoeffs = cv::Mat::zeros(4, 1, CV_64F);
        distCoeffs.at<double>(0) = cameraParams.distortion[0]; // k1
        distCoeffs.at<double>(1) = cameraParams.distortion[1]; // k2
        distCoeffs.at<double>(2) = cameraParams.distortion[2]; // k3 (was p1 in MEI)
        distCoeffs.at<double>(3) = cameraParams.distortion[3]; // k4 (was p2 in MEI)
        
        std::cout << "Fisheye distortion coefficients (k1, k2, k3, k4):" << std::endl << distCoeffs.t() << std::endl;
        std::cout << "Note: Using ALL calibration parameters (no zeros)" << std::endl;
    }
    
    void createUndistortionMaps() {
        cv::Size inputImageSize(cameraParams.image_width, cameraParams.image_height);
        
        // Create a much larger output size for the unwrapped fisheye
        // Fisheye ~180° FOV needs to be spread across a wider format
        // Use approximately 2.5x width and 1.5x height for the panoramic unwrapped view
        outputImageSize.width = static_cast<int>(inputImageSize.width * 2.5);
        outputImageSize.height = static_cast<int>(inputImageSize.height * 1.5);
        
        std::cout << "Creating fisheye undistortion maps:" << std::endl;
        std::cout << "  Input image size: " << inputImageSize << std::endl;
        std::cout << "  Output image size: " << outputImageSize << " (wider for unwrapped view)" << std::endl;
        
        // The key insight: we need to EXPAND the fisheye image, not compress it!
        // For fisheye undistortion, we typically want to INCREASE the focal lengths 
        // to stretch out the compressed fisheye view into a flat perspective
        
        // Try multiple approaches to get the right undistortion
        
        // Approach 1: Adjust camera matrix for larger output and fisheye expansion
        cv::Mat newCameraMatrix1 = cameraMatrix.clone();
        
        // Adjust principal point to center of new larger image
        newCameraMatrix1.at<double>(0, 2) = outputImageSize.width / 2.0;  // cx
        newCameraMatrix1.at<double>(1, 2) = outputImageSize.height / 2.0; // cy
        
        // Increase focal lengths for fisheye expansion
        double expandScale = 2.5;
        newCameraMatrix1.at<double>(0, 0) *= expandScale; // fx
        newCameraMatrix1.at<double>(1, 1) *= expandScale; // fy
        
        // Approach 2: More aggressive expansion with better principal point
        cv::Mat newCameraMatrix2 = cameraMatrix.clone();
        newCameraMatrix2.at<double>(0, 2) = outputImageSize.width / 2.0;  // cx
        newCameraMatrix2.at<double>(1, 2) = outputImageSize.height / 2.0; // cy
        
        double aggressiveScale = 4.0;
        newCameraMatrix2.at<double>(0, 0) *= aggressiveScale; // fx
        newCameraMatrix2.at<double>(1, 1) *= aggressiveScale; // fy
        
        // Approach 3: Try inverting the distortion coefficients
        cv::Mat invertedDistCoeffs = -distCoeffs; // Invert the signs
        
        std::cout << "Testing different approaches for fisheye-to-flat transformation:" << std::endl;
        std::cout << "Original camera matrix:" << std::endl << cameraMatrix << std::endl;
        std::cout << "Expanded camera matrix (scale=" << expandScale << "):" << std::endl << newCameraMatrix1 << std::endl;
        std::cout << "Aggressive expansion (scale=" << aggressiveScale << "):" << std::endl << newCameraMatrix2 << std::endl;
        std::cout << "Original distortion:" << distCoeffs.t() << std::endl;
        std::cout << "Inverted distortion:" << invertedDistCoeffs.t() << std::endl;
        
        // Try approach 1: Expanded focal lengths with original distortion
        try {
            cv::fisheye::initUndistortRectifyMap(
                cameraMatrix, distCoeffs, cv::Mat(),
                newCameraMatrix1, outputImageSize, CV_16SC2,
                mapX, mapY
            );
            std::cout << "✓ Fisheye undistortion maps created with expanded focal lengths and larger output!" << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "Approach 1 failed: " << e.what() << std::endl;
            
            // Try approach 2: More aggressive expansion
            try {
                cv::fisheye::initUndistortRectifyMap(
                    cameraMatrix, distCoeffs, cv::Mat(),
                    newCameraMatrix2, outputImageSize, CV_16SC2,
                    mapX, mapY
                );
                std::cout << "✓ Using aggressive expansion approach with larger output!" << std::endl;
            } catch (const cv::Exception& e2) {
                std::cerr << "Approach 2 failed: " << e2.what() << std::endl;
                
                // Try approach 3: Inverted distortion
                try {
                    cv::fisheye::initUndistortRectifyMap(
                        cameraMatrix, invertedDistCoeffs, cv::Mat(),
                        newCameraMatrix1, outputImageSize, CV_16SC2,
                        mapX, mapY
                    );
                    std::cout << "✓ Using inverted distortion coefficients with larger output!" << std::endl;
                } catch (const cv::Exception& e3) {
                    std::cerr << "All approaches failed. Using standard undistort method..." << std::endl;
                    
                    // Fallback: use standard OpenCV undistort instead of fisheye
                    cv::initUndistortRectifyMap(
                        cameraMatrix, distCoeffs, cv::Mat(),
                        newCameraMatrix1, outputImageSize, CV_16SC2,
                        mapX, mapY
                    );
                    std::cout << "✓ Using standard camera undistortion as fallback with larger output" << std::endl;
                }
            }
        }
    }
    
    cv::Mat undistortImage(const cv::Mat& originalImage) {
        if (!calibrationLoaded || originalImage.empty()) {
            std::cerr << "Cannot undistort: calibration not loaded or image empty" << std::endl;
            return cv::Mat();
        }
        
        std::cout << "Applying undistortion to image:" << std::endl;
        std::cout << "  Input size: " << originalImage.cols << "x" << originalImage.rows << std::endl;
        std::cout << "  Output size: " << outputImageSize.width << "x" << outputImageSize.height << std::endl;
        
        // Create output image with the larger size for unwrapped fisheye
        cv::Mat undistortedImage(outputImageSize, originalImage.type(), cv::Scalar(0, 0, 0));
        
        // Apply the undistortion mapping
        cv::remap(originalImage, undistortedImage, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        
        std::cout << "✓ Undistortion applied successfully to larger output format!" << std::endl;
        return undistortedImage;
    }
    
    void processAndDisplay(const std::string& imagePath) {
        if (!fs::exists(imagePath)) {
            std::cerr << "✗ Image file does not exist: " << imagePath << std::endl;
            return;
        }
        
        std::cout << "\n=== LOADING AND PROCESSING IMAGE ===" << std::endl;
        
        // Load original image
        cv::Mat originalImage = cv::imread(imagePath, cv::IMREAD_COLOR);
        if (originalImage.empty()) {
            std::cerr << "✗ Failed to load image: " << imagePath << std::endl;
            return;
        }
        
        std::cout << "✓ Loaded image: " << originalImage.cols << "x" << originalImage.rows << std::endl;
        
        // Check if image size matches calibration
        if (originalImage.cols != cameraParams.image_width || originalImage.rows != cameraParams.image_height) {
            std::cout << "⚠ WARNING: Image size (" << originalImage.cols << "x" << originalImage.rows 
                      << ") doesn't match calibration size (" << cameraParams.image_width 
                      << "x" << cameraParams.image_height << ")" << std::endl;
        }
        
        // Apply undistortion
        cv::Mat undistortedImage = undistortImage(originalImage);
        if (undistortedImage.empty()) {
            std::cerr << "✗ Failed to undistort image" << std::endl;
            return;
        }
        
        std::cout << "\n=== DISPLAYING RESULTS ===" << std::endl;
        std::cout << "✓ Displaying original and undistorted images in separate windows" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  - Press '1' to focus on original fisheye image" << std::endl;
        std::cout << "  - Press '2' to focus on unwrapped image (extra wide view)" << std::endl;
        std::cout << "  - Press 'c' to show stacked comparison" << std::endl;
        std::cout << "  - Press ESC or 'q' to quit" << std::endl;
        
        // Create windows
        cv::namedWindow("Original Fisheye", cv::WINDOW_NORMAL);
        cv::namedWindow("Undistorted (Unwrapped)", cv::WINDOW_NORMAL);
        
        // Add labels to images
        cv::Mat labeledOriginal = originalImage.clone();
        cv::Mat labeledUndistorted = undistortedImage.clone();
        
        cv::putText(labeledOriginal, "ORIGINAL FISHEYE", cv::Point(30, 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
        cv::putText(labeledUndistorted, "UNDISTORTED (UNWRAPPED)", cv::Point(30, 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        
        // Position windows side by side
        cv::moveWindow("Original Fisheye", 100, 100);
        cv::moveWindow("Undistorted (Unwrapped)", 800, 100);
        
        // Make the undistorted window larger to accommodate the wider format
        cv::resizeWindow("Original Fisheye", 600, 600);
        cv::resizeWindow("Undistorted (Unwrapped)", 1600, 800);  // Much wider for the unwrapped format
        
        // Display both images
        cv::imshow("Original Fisheye", labeledOriginal);
        cv::imshow("Undistorted (Unwrapped)", labeledUndistorted);
        
        // Create comparison view - since images are different sizes, stack vertically
        cv::Mat comparison;
        cv::Mat resizedOriginal, resizedUndistorted;
        
        // Resize both to have the same width for vertical stacking
        int comparisonWidth = std::max(originalImage.cols, undistortedImage.cols);
        cv::resize(originalImage, resizedOriginal, cv::Size(comparisonWidth, 
                  originalImage.rows * comparisonWidth / originalImage.cols));
        cv::resize(undistortedImage, resizedUndistorted, cv::Size(comparisonWidth, 
                  undistortedImage.rows * comparisonWidth / undistortedImage.cols));
        
        cv::vconcat(resizedOriginal, resizedUndistorted, comparison);
        
        cv::putText(comparison, "ORIGINAL FISHEYE", cv::Point(30, 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
        cv::putText(comparison, "UNDISTORTED (UNWRAPPED)", cv::Point(30, resizedOriginal.rows + 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
        
        // Draw horizontal line to separate the images
        cv::line(comparison, cv::Point(0, resizedOriginal.rows), 
                 cv::Point(comparison.cols, resizedOriginal.rows), cv::Scalar(255, 255, 255), 3);
        
        // Interactive loop
        while (true) {
            int key = cv::waitKey(0);
            
            if (key == 27 || key == 'q' || key == 'Q') { // ESC or 'q'
                break;
            } else if (key == '1') {
                // Focus on original image - make it larger
                cv::destroyWindow("Undistorted (Unwrapped)");
                cv::destroyWindow("Comparison");
                cv::namedWindow("Original Fisheye - LARGE", cv::WINDOW_NORMAL);
                cv::resizeWindow("Original Fisheye - LARGE", 1000, 1000);
                cv::imshow("Original Fisheye - LARGE", labeledOriginal);
                std::cout << "Focused on original fisheye image. Press '2' for undistorted or 'c' for comparison." << std::endl;
            } else if (key == '2') {
                // Focus on undistorted image - make it even larger and wider for the unwrapped format
                cv::destroyWindow("Original Fisheye");
                cv::destroyWindow("Original Fisheye - LARGE");
                cv::destroyWindow("Comparison");
                cv::namedWindow("Undistorted - LARGE", cv::WINDOW_NORMAL);
                cv::resizeWindow("Undistorted - LARGE", 1800, 800);  // Much wider for unwrapped format
                cv::imshow("Undistorted - LARGE", labeledUndistorted);
                std::cout << "Focused on unwrapped fisheye (extra wide view). Press '1' for original or 'c' for comparison." << std::endl;
            } else if (key == 'c' || key == 'C') {
                // Show stacked comparison (original top, unwrapped bottom)
                cv::destroyWindow("Original Fisheye");
                cv::destroyWindow("Original Fisheye - LARGE");
                cv::destroyWindow("Undistorted - LARGE");
                cv::destroyWindow("Undistorted (Unwrapped)");
                cv::namedWindow("Comparison", cv::WINDOW_NORMAL);
                cv::resizeWindow("Comparison", 1200, 1000);  // Taller for vertical stacking
                cv::imshow("Comparison", comparison);
                std::cout << "Showing stacked comparison (original top, unwrapped bottom). Press '1' or '2' for individual views." << std::endl;
            }
        }
        
        cv::destroyAllWindows();
        
        std::cout << "✓ Done!" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <image_path>" << std::endl;
        std::cerr << "Example: " << argv[0] << " /path/to/fisheye/image.png" << std::endl;
        return 1;
    }
    
    std::string imagePath = argv[1];
    
    FisheyeUndistorter undistorter;
    
    // Load fisheye calibration
    if (!undistorter.loadCalibration()) {
        std::cerr << "ERROR: Failed to load calibration data! Cannot proceed without calibration." << std::endl;
        std::cerr << "Make sure kitti360_calibration/image_02.yaml exists and is readable." << std::endl;
        return 1;
    }
    
    // Process and display the image
    undistorter.processAndDisplay(imagePath);
    
    return 0;
}