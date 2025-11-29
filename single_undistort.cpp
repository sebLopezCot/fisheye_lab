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
    cv::Mat originalImage;     // Store original image for real-time processing
    bool calibrationLoaded;
    
    // Interactive parameters
    double currentFocalScale;
    double currentWidthMultiplier;
    double currentHeightMultiplier;
    
    // Interactive calibration parameters
    cv::Mat adjustedCameraMatrix, adjustedDistCoeffs;
    
public:
    FisheyeUndistorter() : calibrationLoaded(false), currentFocalScale(5.0), 
                           currentWidthMultiplier(4.0), currentHeightMultiplier(2.0) {}
    
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
            
            // Initialize adjustable parameters with loaded values
            adjustedCameraMatrix = cameraMatrix.clone();
            adjustedDistCoeffs = distCoeffs.clone();
            
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
        // For ultra-flat projection, we need extreme width to spread angular changes linearly
        // Use approximately 4x width and 2x height for ultra-wide panoramic view
        outputImageSize.width = static_cast<int>(inputImageSize.width * 4.0);
        outputImageSize.height = static_cast<int>(inputImageSize.height * 2.0);
        
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
        
        // For ultra-flat projection, use much more aggressive focal length scaling
        double expandScale = 5.0;  // Much higher for flatter projection
        newCameraMatrix1.at<double>(0, 0) *= expandScale; // fx
        newCameraMatrix1.at<double>(1, 1) *= expandScale; // fy
        
        // Approach 2: Even more aggressive expansion for ultra-flat result
        cv::Mat newCameraMatrix2 = cameraMatrix.clone();
        newCameraMatrix2.at<double>(0, 2) = outputImageSize.width / 2.0;  // cx
        newCameraMatrix2.at<double>(1, 2) = outputImageSize.height / 2.0; // cy
        
        double ultraFlatScale = 8.0;  // Ultra-aggressive scaling
        newCameraMatrix2.at<double>(0, 0) *= ultraFlatScale; // fx
        newCameraMatrix2.at<double>(1, 1) *= ultraFlatScale; // fy
        
        // Approach 3: Try inverting the distortion coefficients
        cv::Mat invertedDistCoeffs = -distCoeffs; // Invert the signs
        
        std::cout << "Testing different approaches for ultra-flat fisheye transformation:" << std::endl;
        std::cout << "Original camera matrix:" << std::endl << cameraMatrix << std::endl;
        std::cout << "Expanded camera matrix (scale=" << expandScale << "):" << std::endl << newCameraMatrix1 << std::endl;
        std::cout << "Ultra-flat expansion (scale=" << ultraFlatScale << "):" << std::endl << newCameraMatrix2 << std::endl;
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
        std::cout << "  Full unwrapped size: " << outputImageSize.width << "x" << outputImageSize.height << std::endl;
        
        // Create output image with the larger size for unwrapped fisheye
        cv::Mat undistortedImageFull(outputImageSize, originalImage.type(), cv::Scalar(0, 0, 0));
        
        // Apply the undistortion mapping
        cv::remap(originalImage, undistortedImageFull, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        
        // Scale down to a more screen-friendly size while preserving aspect ratio
        cv::Mat undistortedImage = scaleForDisplay(undistortedImageFull);
        
        std::cout << "✓ Undistortion applied and scaled for display!" << std::endl;
        return undistortedImage;
    }
    
    cv::Mat scaleForDisplay(const cv::Mat& fullSizeImage) {
        // Calculate a reasonable display size (target max width ~1800px)
        double targetMaxWidth = 1800.0;
        double scale = targetMaxWidth / fullSizeImage.cols;
        
        // Make sure we don't scale up, only down
        if (scale > 1.0) scale = 1.0;
        
        int displayWidth = static_cast<int>(fullSizeImage.cols * scale);
        int displayHeight = static_cast<int>(fullSizeImage.rows * scale);
        
        cv::Mat scaledImage;
        cv::resize(fullSizeImage, scaledImage, cv::Size(displayWidth, displayHeight), 0, 0, cv::INTER_AREA);
        
        std::cout << "  Scaled to display size: " << displayWidth << "x" << displayHeight 
                  << " (scale=" << scale << ")" << std::endl;
        
        return scaledImage;
    }
    
    void updateUndistortionMaps() {
        cv::Size inputImageSize(cameraParams.image_width, cameraParams.image_height);
        
        // Create output size based on current multipliers
        outputImageSize.width = static_cast<int>(inputImageSize.width * currentWidthMultiplier);
        outputImageSize.height = static_cast<int>(inputImageSize.height * currentHeightMultiplier);
        
        // Create new camera matrix with current focal scale
        cv::Mat newCameraMatrix = cameraMatrix.clone();
        newCameraMatrix.at<double>(0, 2) = outputImageSize.width / 2.0;  // cx
        newCameraMatrix.at<double>(1, 2) = outputImageSize.height / 2.0; // cy
        newCameraMatrix.at<double>(0, 0) *= currentFocalScale; // fx
        newCameraMatrix.at<double>(1, 1) *= currentFocalScale; // fy
        
        // Create undistortion maps with current parameters (using adjusted calibration)
        try {
            cv::fisheye::initUndistortRectifyMap(
                adjustedCameraMatrix, adjustedDistCoeffs, cv::Mat(),
                newCameraMatrix, outputImageSize, CV_16SC2,
                mapX, mapY
            );
        } catch (const cv::Exception& e) {
            std::cerr << "Fisheye undistortion failed, using standard undistortion..." << std::endl;
            cv::initUndistortRectifyMap(
                adjustedCameraMatrix, adjustedDistCoeffs, cv::Mat(),
                newCameraMatrix, outputImageSize, CV_16SC2,
                mapX, mapY
            );
        }
    }
    
    cv::Mat processWithCurrentParams() {
        if (!calibrationLoaded || originalImage.empty()) {
            return cv::Mat();
        }
        
        // Update undistortion maps with current parameters
        updateUndistortionMaps();
        
        // Apply undistortion
        cv::Mat undistortedImageFull(outputImageSize, originalImage.type(), cv::Scalar(0, 0, 0));
        cv::remap(originalImage, undistortedImageFull, mapX, mapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        
        // Scale for display
        return scaleForDisplay(undistortedImageFull);
    }
    
    void processAndDisplay(const std::string& imagePath) {
        if (!fs::exists(imagePath)) {
            std::cerr << "✗ Image file does not exist: " << imagePath << std::endl;
            return;
        }
        
        std::cout << "\n=== LOADING AND PROCESSING IMAGE ===" << std::endl;
        
        // Load and store original image
        originalImage = cv::imread(imagePath, cv::IMREAD_COLOR);
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
        
        std::cout << "\n=== INTERACTIVE FISHEYE CALIBRATION TUNING ===" << std::endl;
        std::cout << "Use trackbars to adjust calibration parameters in real-time:" << std::endl;
        std::cout << "  PROJECTION CONTROLS:" << std::endl;
        std::cout << "    - Focal Scale: How much to expand the fisheye (higher = flatter)" << std::endl;
        std::cout << "    - Width/Height: Output image dimensions" << std::endl;
        std::cout << "  CALIBRATION TUNING:" << std::endl;
        std::cout << "    - k1, k2: Radial distortion coefficients" << std::endl;
        std::cout << "    - k3, k4: Additional fisheye distortion" << std::endl;
        std::cout << "    - fx, fy: Camera focal lengths" << std::endl;
        std::cout << "Press ESC to quit" << std::endl;
        
        // Create windows
        cv::namedWindow("Original Fisheye", cv::WINDOW_NORMAL);
        cv::namedWindow("Interactive Undistorted", cv::WINDOW_NORMAL);
        cv::namedWindow("Projection Controls", cv::WINDOW_NORMAL);
        cv::namedWindow("Calibration Tuning", cv::WINDOW_NORMAL);
        
        // Create placeholder images for control windows
        cv::Mat projControlsImage = cv::Mat::zeros(180, 600, CV_8UC3);
        cv::putText(projControlsImage, "PROJECTION CONTROLS", cv::Point(150, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
        cv::putText(projControlsImage, "Adjust output format and expansion:", cv::Point(50, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
        cv::imshow("Projection Controls", projControlsImage);
        
        cv::Mat calibControlsImage = cv::Mat::zeros(280, 600, CV_8UC3);
        cv::putText(calibControlsImage, "CALIBRATION TUNING", cv::Point(150, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 255), 2);
        cv::putText(calibControlsImage, "Fine-tune fisheye distortion model:", cv::Point(50, 60), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 200), 1);
        cv::putText(calibControlsImage, "k1,k2: Main radial distortion", cv::Point(50, 90), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 150), 1);
        cv::putText(calibControlsImage, "k3,k4: Additional fisheye correction", cv::Point(50, 110), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 150), 1);
        cv::putText(calibControlsImage, "fx,fy: Focal length scaling", cv::Point(50, 130), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(150, 150, 150), 1);
        cv::imshow("Calibration Tuning", calibControlsImage);
        
        // Position windows
        cv::moveWindow("Original Fisheye", 50, 50);
        cv::moveWindow("Interactive Undistorted", 600, 50);
        cv::moveWindow("Projection Controls", 50, 650);
        cv::moveWindow("Calibration Tuning", 700, 650);
        
        // Resize windows
        cv::resizeWindow("Original Fisheye", 500, 500);
        cv::resizeWindow("Interactive Undistorted", 1000, 500);
        cv::resizeWindow("Projection Controls", 600, 220);
        cv::resizeWindow("Calibration Tuning", 600, 320);
        
        // Create projection control trackbars
        int focalScale = static_cast<int>(currentFocalScale * 10);
        int widthMult = static_cast<int>(currentWidthMultiplier * 10);
        int heightMult = static_cast<int>(currentHeightMultiplier * 10);
        
        cv::createTrackbar("Focal Scale x10", "Projection Controls", &focalScale, 150, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->currentFocalScale = val / 10.0;
                              self->updateDisplay();
                          }, this);
        
        cv::createTrackbar("Width Mult x10", "Projection Controls", &widthMult, 100, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->currentWidthMultiplier = std::max(1.0, val / 10.0);
                              self->updateDisplay();
                          }, this);
        
        cv::createTrackbar("Height Mult x10", "Projection Controls", &heightMult, 50, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->currentHeightMultiplier = std::max(1.0, val / 10.0);
                              self->updateDisplay();
                          }, this);
        
        // Create calibration tuning trackbars
        // For distortion coefficients, use offset to allow negative values
        int k1_offset = static_cast<int>((adjustedDistCoeffs.at<double>(0) + 2.0) * 1000); // Range: -2.0 to 2.0
        int k2_offset = static_cast<int>((adjustedDistCoeffs.at<double>(1) + 5.0) * 100);  // Range: -5.0 to 5.0
        int k3_offset = static_cast<int>((adjustedDistCoeffs.at<double>(2) + 0.01) * 10000); // Range: -0.01 to 0.01
        int k4_offset = static_cast<int>((adjustedDistCoeffs.at<double>(3) + 0.01) * 10000); // Range: -0.01 to 0.01
        
        cv::createTrackbar("k1 x1000+2000", "Calibration Tuning", &k1_offset, 4000, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->adjustedDistCoeffs.at<double>(0) = (val / 1000.0) - 2.0;
                              self->updateDisplay();
                          }, this);
        
        cv::createTrackbar("k2 x100+500", "Calibration Tuning", &k2_offset, 1000, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->adjustedDistCoeffs.at<double>(1) = (val / 100.0) - 5.0;
                              self->updateDisplay();
                          }, this);
        
        cv::createTrackbar("k3 x10000+100", "Calibration Tuning", &k3_offset, 200, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->adjustedDistCoeffs.at<double>(2) = (val / 10000.0) - 0.01;
                              self->updateDisplay();
                          }, this);
        
        cv::createTrackbar("k4 x10000+100", "Calibration Tuning", &k4_offset, 200, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->adjustedDistCoeffs.at<double>(3) = (val / 10000.0) - 0.01;
                              self->updateDisplay();
                          }, this);
        
        // Camera focal length adjustments (as percentages of original)
        int fx_percent = 100; // 100% = original value
        int fy_percent = 100;
        
        cv::createTrackbar("fx percent", "Calibration Tuning", &fx_percent, 200, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->adjustedCameraMatrix.at<double>(0, 0) = self->cameraMatrix.at<double>(0, 0) * (val / 100.0);
                              self->updateDisplay();
                          }, this);
        
        cv::createTrackbar("fy percent", "Calibration Tuning", &fy_percent, 200, 
                          [](int val, void* userdata) {
                              auto* self = static_cast<FisheyeUndistorter*>(userdata);
                              self->adjustedCameraMatrix.at<double>(1, 1) = self->cameraMatrix.at<double>(1, 1) * (val / 100.0);
                              self->updateDisplay();
                          }, this);
        
        // Display original image
        cv::Mat labeledOriginal = originalImage.clone();
        cv::putText(labeledOriginal, "ORIGINAL FISHEYE", cv::Point(30, 40), 
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);
        cv::imshow("Original Fisheye", labeledOriginal);
        
        // Initial undistortion
        updateDisplay();
        
        // Wait for user input
        while (true) {
            int key = cv::waitKey(30);
            if (key == 27) { // ESC
                break;
            }
        }
        
        cv::destroyAllWindows();
        std::cout << "✓ Done! Final parameters:" << std::endl;
        std::cout << "  Projection: Focal=" << currentFocalScale 
                  << ", Width=" << currentWidthMultiplier 
                  << ", Height=" << currentHeightMultiplier << std::endl;
        std::cout << "  Calibration:" << std::endl;
        std::cout << "    Distortion: k1=" << adjustedDistCoeffs.at<double>(0) 
                  << ", k2=" << adjustedDistCoeffs.at<double>(1)
                  << ", k3=" << adjustedDistCoeffs.at<double>(2)
                  << ", k4=" << adjustedDistCoeffs.at<double>(3) << std::endl;
        std::cout << "    Focal lengths: fx=" << adjustedCameraMatrix.at<double>(0, 0)
                  << ", fy=" << adjustedCameraMatrix.at<double>(1, 1) << std::endl;
    }
    
    void updateDisplay() {
        cv::Mat undistorted = processWithCurrentParams();
        if (!undistorted.empty()) {
            // Add parameter info to the image
            cv::Mat labeledUndistorted = undistorted.clone();
            
            std::string projText = "Proj: F=" + std::to_string(currentFocalScale).substr(0,4) + 
                                 " W=" + std::to_string(currentWidthMultiplier).substr(0,4) + 
                                 " H=" + std::to_string(currentHeightMultiplier).substr(0,4);
            std::string calibText = "Calib: k1=" + std::to_string(adjustedDistCoeffs.at<double>(0)).substr(0,6) +
                                  " k2=" + std::to_string(adjustedDistCoeffs.at<double>(1)).substr(0,6);
            std::string calibText2 = "k3=" + std::to_string(adjustedDistCoeffs.at<double>(2)).substr(0,7) +
                                   " k4=" + std::to_string(adjustedDistCoeffs.at<double>(3)).substr(0,7);
            
            cv::putText(labeledUndistorted, "INTERACTIVE CALIBRATION TUNING", cv::Point(30, 40), 
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            cv::putText(labeledUndistorted, projText, cv::Point(30, 80), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            cv::putText(labeledUndistorted, calibText, cv::Point(30, 110), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
            cv::putText(labeledUndistorted, calibText2, cv::Point(30, 140), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
            
            cv::imshow("Interactive Undistorted", labeledUndistorted);
        }
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