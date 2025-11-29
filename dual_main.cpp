#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <opencv2/opencv.hpp>
#include "kitti360_calibration/load_calibration.h"
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <set>

namespace fs = std::filesystem;

struct StereoImageData {
    // Left eye data
    SDL_Texture* leftTexture;
    SDL_Surface* leftSurface;
    std::string leftFilename;
    std::atomic<bool> leftSurfaceLoaded;
    std::atomic<bool> leftTextureCreated;
    
    // Right eye data
    SDL_Texture* rightTexture;
    SDL_Surface* rightSurface;
    std::string rightFilename;
    std::atomic<bool> rightSurfaceLoaded;
    std::atomic<bool> rightTextureCreated;
    
    // Common data
    std::string baseName; // e.g., "0000007667"
    
    StereoImageData() : leftTexture(nullptr), leftSurface(nullptr), leftSurfaceLoaded(false), leftTextureCreated(false),
                        rightTexture(nullptr), rightSurface(nullptr), rightSurfaceLoaded(false), rightTextureCreated(false) {}
    
    ~StereoImageData() {
        if (leftTexture) {
            SDL_DestroyTexture(leftTexture);
        }
        if (leftSurface) {
            SDL_FreeSurface(leftSurface);
        }
        if (rightTexture) {
            SDL_DestroyTexture(rightTexture);
        }
        if (rightSurface) {
            SDL_FreeSurface(rightSurface);
        }
    }
};

class StereoFisheyeViewer {
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    std::vector<std::unique_ptr<StereoImageData>> stereoPairs;
    int currentIndex;
    int windowWidth, windowHeight;
    bool running;
    
    // Calibration and undistortion
    kitti360::FisheyeParams leftCameraParams;  // image_02
    kitti360::FisheyeParams rightCameraParams; // image_03
    cv::Mat leftCameraMatrix, leftDistCoeffs;
    cv::Mat rightCameraMatrix, rightDistCoeffs;
    cv::Mat leftMapX, leftMapY, rightMapX, rightMapY;
    cv::Size outputImageSize;  // Size for the unwrapped output images
    cv::Size displayImageSize; // Size for screen-friendly display
    bool calibrationLoaded;
    
    // Background loading
    std::vector<std::thread> backgroundLoaders;
    std::mutex imagesMutex;
    std::atomic<bool> backgroundLoadingComplete;
    std::atomic<size_t> nextImageToLoad;
    const int INITIAL_LOAD_COUNT = 10;
    const int NUM_LOADING_THREADS = 4;
    
public:
    StereoFisheyeViewer() : window(nullptr), renderer(nullptr), currentIndex(0), 
                            windowWidth(1800), windowHeight(900), running(true), 
                            calibrationLoaded(false), backgroundLoadingComplete(false), 
                            nextImageToLoad(0) {}
    
    ~StereoFisheyeViewer() {
        cleanup();
    }
    
    bool initialize() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            std::cerr << "SDL could not initialize! SDL Error: " << SDL_GetError() << std::endl;
            return false;
        }
        
        if (!(IMG_Init(IMG_INIT_JPG | IMG_INIT_PNG) & (IMG_INIT_JPG | IMG_INIT_PNG))) {
            std::cerr << "SDL_image could not initialize! SDL_image Error: " << IMG_GetError() << std::endl;
            return false;
        }
        
        window = SDL_CreateWindow("Ultra-Flat Dual Fisheye Unwrapped Viewer (Screen-Scaled) - Left & Right",
                                  SDL_WINDOWPOS_CENTERED,
                                  SDL_WINDOWPOS_CENTERED,
                                  windowWidth, windowHeight,
                                  SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        
        if (!window) {
            std::cerr << "Window could not be created! SDL Error: " << SDL_GetError() << std::endl;
            return false;
        }
        
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (!renderer) {
            std::cerr << "Renderer could not be created! SDL Error: " << SDL_GetError() << std::endl;
            return false;
        }
        
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        
        return true;
    }
    
    bool loadCalibration() {
        try {
            std::cout << "=== LOADING DUAL FISHEYE CALIBRATION PARAMETERS ===" << std::endl;
            
            // Load fisheye parameters for left (image_02) and right (image_03) cameras
            leftCameraParams = kitti360::loadFisheyeParams("kitti360_calibration/image_02.yaml");
            rightCameraParams = kitti360::loadFisheyeParams("kitti360_calibration/image_03.yaml");
            
            std::cout << "✓ Successfully loaded calibration files" << std::endl;
            std::cout << "Left camera (image_02): " << leftCameraParams.camera_name << std::endl;
            std::cout << "  Image size: " << leftCameraParams.image_width << "x" << leftCameraParams.image_height << std::endl;
            std::cout << "  Xi: " << leftCameraParams.xi << std::endl;
            std::cout << "  Distortion: k1=" << leftCameraParams.distortion[0] << ", k2=" << leftCameraParams.distortion[1] 
                      << ", p1=" << leftCameraParams.distortion[2] << ", p2=" << leftCameraParams.distortion[3] << std::endl;
            
            std::cout << "Right camera (image_03): " << rightCameraParams.camera_name << std::endl;
            std::cout << "  Image size: " << rightCameraParams.image_width << "x" << rightCameraParams.image_height << std::endl;
            std::cout << "  Xi: " << rightCameraParams.xi << std::endl;
            std::cout << "  Distortion: k1=" << rightCameraParams.distortion[0] << ", k2=" << rightCameraParams.distortion[1] 
                      << ", p1=" << rightCameraParams.distortion[2] << ", p2=" << rightCameraParams.distortion[3] << std::endl;
            
            // Convert fisheye parameters to OpenCV format
            setupCameraMatrices();
            
            // Create undistortion maps with wider output format
            createUndistortionMaps();
            
            calibrationLoaded = true;
            std::cout << "✓ Dual camera calibration loaded and undistortion maps created successfully!" << std::endl;
            std::cout << "==================================================================" << std::endl;
            return true;
            
        } catch (const std::exception& e) {
            std::cerr << "✗ CRITICAL ERROR: Failed to load dual calibration: " << e.what() << std::endl;
            std::cerr << "✗ Make sure both kitti360_calibration/image_02.yaml and image_03.yaml exist" << std::endl;
            calibrationLoaded = false;
            return false;
        }
    }
    
    void setupCameraMatrices() {
        // Setup left camera matrix from fisheye parameters
        leftCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        leftCameraMatrix.at<double>(0, 0) = leftCameraParams.projection[0]; // gamma1 (fx)
        leftCameraMatrix.at<double>(1, 1) = leftCameraParams.projection[1]; // gamma2 (fy)
        leftCameraMatrix.at<double>(0, 2) = leftCameraParams.projection[2]; // u0 (cx)
        leftCameraMatrix.at<double>(1, 2) = leftCameraParams.projection[3]; // v0 (cy)
        
        // Setup right camera matrix from fisheye parameters
        rightCameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        rightCameraMatrix.at<double>(0, 0) = rightCameraParams.projection[0]; // gamma1 (fx)
        rightCameraMatrix.at<double>(1, 1) = rightCameraParams.projection[1]; // gamma2 (fy)
        rightCameraMatrix.at<double>(0, 2) = rightCameraParams.projection[2]; // u0 (cx)
        rightCameraMatrix.at<double>(1, 2) = rightCameraParams.projection[3]; // v0 (cy)
        
        std::cout << "Left camera matrix:" << std::endl << leftCameraMatrix << std::endl;
        std::cout << "Right camera matrix:" << std::endl << rightCameraMatrix << std::endl;
        
        // For fisheye model, use ALL 4 distortion parameters from calibration
        // Map MEI model parameters to OpenCV fisheye model: k1, k2, p1->k3, p2->k4
        leftDistCoeffs = cv::Mat::zeros(4, 1, CV_64F);
        leftDistCoeffs.at<double>(0) = leftCameraParams.distortion[0]; // k1
        leftDistCoeffs.at<double>(1) = leftCameraParams.distortion[1]; // k2
        leftDistCoeffs.at<double>(2) = leftCameraParams.distortion[2]; // k3 (was p1)
        leftDistCoeffs.at<double>(3) = leftCameraParams.distortion[3]; // k4 (was p2)
        
        rightDistCoeffs = cv::Mat::zeros(4, 1, CV_64F);
        rightDistCoeffs.at<double>(0) = rightCameraParams.distortion[0]; // k1
        rightDistCoeffs.at<double>(1) = rightCameraParams.distortion[1]; // k2
        rightDistCoeffs.at<double>(2) = rightCameraParams.distortion[2]; // k3 (was p1)
        rightDistCoeffs.at<double>(3) = rightCameraParams.distortion[3]; // k4 (was p2)
        
        std::cout << "Left distortion coefficients (k1, k2, k3, k4): " << leftDistCoeffs.t() << std::endl;
        std::cout << "Right distortion coefficients (k1, k2, k3, k4): " << rightDistCoeffs.t() << std::endl;
        std::cout << "Note: Using ALL calibration parameters (no zeros)" << std::endl;
    }
    
    void createUndistortionMaps() {
        cv::Size inputImageSize(leftCameraParams.image_width, leftCameraParams.image_height);
        
        // Create a much larger output size for the unwrapped fisheye
        // For ultra-flat projection, we need extreme width to spread angular changes linearly
        outputImageSize.width = static_cast<int>(inputImageSize.width * 4.0);
        outputImageSize.height = static_cast<int>(inputImageSize.height * 2.0);
        
        std::cout << "Creating dual fisheye undistortion maps:" << std::endl;
        std::cout << "  Input image size: " << inputImageSize << std::endl;
        std::cout << "  Output image size: " << outputImageSize << " (wider for unwrapped view)" << std::endl;
        
        // Create new camera matrices for larger output and fisheye expansion
        cv::Mat newLeftCameraMatrix = leftCameraMatrix.clone();
        cv::Mat newRightCameraMatrix = rightCameraMatrix.clone();
        
        // Adjust principal points to center of new larger image
        newLeftCameraMatrix.at<double>(0, 2) = outputImageSize.width / 2.0;  // cx
        newLeftCameraMatrix.at<double>(1, 2) = outputImageSize.height / 2.0; // cy
        newRightCameraMatrix.at<double>(0, 2) = outputImageSize.width / 2.0;  // cx
        newRightCameraMatrix.at<double>(1, 2) = outputImageSize.height / 2.0; // cy
        
        // For ultra-flat projection, use much more aggressive focal length scaling
        double expandScale = 5.0;  // Much higher for flatter projection
        newLeftCameraMatrix.at<double>(0, 0) *= expandScale; // fx
        newLeftCameraMatrix.at<double>(1, 1) *= expandScale; // fy
        newRightCameraMatrix.at<double>(0, 0) *= expandScale; // fx
        newRightCameraMatrix.at<double>(1, 1) *= expandScale; // fy
        
        std::cout << "Camera matrix scaling factor: " << expandScale << std::endl;
        std::cout << "New left camera matrix:" << std::endl << newLeftCameraMatrix << std::endl;
        std::cout << "New right camera matrix:" << std::endl << newRightCameraMatrix << std::endl;
        
        // Create undistortion maps for left camera using fisheye model
        try {
            cv::fisheye::initUndistortRectifyMap(
                leftCameraMatrix, leftDistCoeffs, cv::Mat(),
                newLeftCameraMatrix, outputImageSize, CV_16SC2,
                leftMapX, leftMapY
            );
            std::cout << "✓ Left camera undistortion maps created successfully!" << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "Left camera undistortion failed: " << e.what() << std::endl;
            std::cerr << "Falling back to standard undistortion for left camera..." << std::endl;
            cv::initUndistortRectifyMap(
                leftCameraMatrix, leftDistCoeffs, cv::Mat(),
                newLeftCameraMatrix, outputImageSize, CV_16SC2,
                leftMapX, leftMapY
            );
        }
        
        // Create undistortion maps for right camera using fisheye model
        try {
            cv::fisheye::initUndistortRectifyMap(
                rightCameraMatrix, rightDistCoeffs, cv::Mat(),
                newRightCameraMatrix, outputImageSize, CV_16SC2,
                rightMapX, rightMapY
            );
            std::cout << "✓ Right camera undistortion maps created successfully!" << std::endl;
        } catch (const cv::Exception& e) {
            std::cerr << "Right camera undistortion failed: " << e.what() << std::endl;
            std::cerr << "Falling back to standard undistortion for right camera..." << std::endl;
            cv::initUndistortRectifyMap(
                rightCameraMatrix, rightDistCoeffs, cv::Mat(),
                newRightCameraMatrix, outputImageSize, CV_16SC2,
                rightMapX, rightMapY
            );
        }
        
        std::cout << "✓ Dual fisheye undistortion maps created successfully!" << std::endl;
        
        // Calculate display size (scale down for screen-friendly viewing)
        double targetMaxWidth = 800.0;  // Target width for each camera view (half of total window)
        double scale = targetMaxWidth / outputImageSize.width;
        if (scale > 1.0) scale = 1.0;  // Don't scale up
        
        displayImageSize.width = static_cast<int>(outputImageSize.width * scale);
        displayImageSize.height = static_cast<int>(outputImageSize.height * scale);
        
        std::cout << "Display size (scaled): " << displayImageSize.width << "x" << displayImageSize.height 
                  << " (scale=" << scale << ")" << std::endl;
    }
    
    SDL_Surface* undistortImage(SDL_Surface* originalSurface, bool isLeftCamera) {
        if (!calibrationLoaded || !originalSurface) {
            return nullptr;
        }
        
        // Convert SDL surface to OpenCV Mat
        cv::Mat originalMat = sdlSurfaceToMat(originalSurface);
        if (originalMat.empty()) {
            return nullptr;
        }
        
        // Create output image with the larger size for unwrapped fisheye
        cv::Mat undistortedMatFull(outputImageSize, originalMat.type(), cv::Scalar(0, 0, 0));
        
        // Apply undistortion to larger output format
        if (isLeftCamera) {
            cv::remap(originalMat, undistortedMatFull, leftMapX, leftMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        } else {
            cv::remap(originalMat, undistortedMatFull, rightMapX, rightMapY, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        }
        
        // Scale down to display size while preserving aspect ratio
        cv::Mat undistortedMat;
        cv::resize(undistortedMatFull, undistortedMat, displayImageSize, 0, 0, cv::INTER_AREA);
        
        // Convert back to SDL surface
        return matToSdlSurface(undistortedMat);
    }
    
    cv::Mat sdlSurfaceToMat(SDL_Surface* surface) {
        if (!surface) return cv::Mat();
        
        // Convert SDL surface format if needed
        SDL_Surface* rgbSurface = SDL_ConvertSurfaceFormat(surface, SDL_PIXELFORMAT_RGB24, 0);
        if (!rgbSurface) {
            std::cerr << "Failed to convert surface format: " << SDL_GetError() << std::endl;
            return cv::Mat();
        }
        
        // Create OpenCV Mat from surface data
        cv::Mat mat(rgbSurface->h, rgbSurface->w, CV_8UC3, rgbSurface->pixels, rgbSurface->pitch);
        
        // OpenCV uses BGR, SDL uses RGB, so convert
        cv::Mat bgrMat;
        cv::cvtColor(mat, bgrMat, cv::COLOR_RGB2BGR);
        
        // Make a copy since we'll free the surface
        cv::Mat result = bgrMat.clone();
        
        SDL_FreeSurface(rgbSurface);
        return result;
    }
    
    SDL_Surface* matToSdlSurface(const cv::Mat& mat) {
        if (mat.empty()) return nullptr;
        
        // Convert BGR to RGB
        cv::Mat rgbMat;
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
        
        // Create SDL surface
        SDL_Surface* surface = SDL_CreateRGBSurfaceFrom(
            rgbMat.data, rgbMat.cols, rgbMat.rows, 24, rgbMat.step,
            0x000000FF, 0x0000FF00, 0x00FF0000, 0
        );
        
        if (!surface) {
            std::cerr << "Failed to create SDL surface: " << SDL_GetError() << std::endl;
            return nullptr;
        }
        
        // Make a copy since the original data will go out of scope
        SDL_Surface* copy = SDL_ConvertSurface(surface, surface->format, 0);
        SDL_FreeSurface(surface);
        
        return copy;
    }
    
    bool loadStereoPairs(const std::string& leftDir, const std::string& rightDir) {
        std::set<std::string> leftBaseNames;
        std::set<std::string> rightBaseNames;
        
        // Scan left directory
        try {
            for (const auto& entry : fs::directory_iterator(leftDir)) {
                if (entry.is_regular_file()) {
                    std::string extension = entry.path().extension().string();
                    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
                    
                    if (extension == ".jpg" || extension == ".jpeg" || extension == ".png") {
                        std::string baseName = entry.path().stem().string();
                        leftBaseNames.insert(baseName);
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error reading left directory: " << e.what() << std::endl;
            return false;
        }
        
        // Scan right directory
        try {
            for (const auto& entry : fs::directory_iterator(rightDir)) {
                if (entry.is_regular_file()) {
                    std::string extension = entry.path().extension().string();
                    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
                    
                    if (extension == ".jpg" || extension == ".jpeg" || extension == ".png") {
                        std::string baseName = entry.path().stem().string();
                        rightBaseNames.insert(baseName);
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error reading right directory: " << e.what() << std::endl;
            return false;
        }
        
        // Find matching pairs
        std::vector<std::string> matchingPairs;
        for (const auto& leftName : leftBaseNames) {
            if (rightBaseNames.find(leftName) != rightBaseNames.end()) {
                matchingPairs.push_back(leftName);
            }
        }
        
        if (matchingPairs.empty()) {
            std::cerr << "No matching stereo pairs found between directories" << std::endl;
            return false;
        }
        
        // Sort pairs in ascending order
        std::sort(matchingPairs.begin(), matchingPairs.end());
        
        // Check if we have too many pairs and ask user
        if (matchingPairs.size() > 1000) {
            std::cout << "\nFound " << matchingPairs.size() << " stereo pairs. This is a large dataset." << std::endl;
            std::cout << "Loading all pairs may use significant memory and time." << std::endl;
            std::cout << "Do you want to:" << std::endl;
            std::cout << "  1. Load all " << matchingPairs.size() << " pairs" << std::endl;
            std::cout << "  2. Load only the first 1000 pairs" << std::endl;
            std::cout << "Enter your choice (1 or 2): ";
            
            std::string choice;
            std::getline(std::cin, choice);
            
            if (choice == "2") {
                matchingPairs.resize(1000);
                std::cout << "Limiting to first 1000 pairs." << std::endl;
            } else {
                std::cout << "Loading all " << matchingPairs.size() << " pairs." << std::endl;
            }
        }
        
        // Initialize stereo pair data structures
        stereoPairs.resize(matchingPairs.size());
        for (size_t i = 0; i < stereoPairs.size(); ++i) {
            stereoPairs[i] = std::make_unique<StereoImageData>();
            stereoPairs[i]->baseName = matchingPairs[i];
            
            // Construct full file paths (assume .png extension for now)
            stereoPairs[i]->leftFilename = leftDir + "/" + matchingPairs[i] + ".png";
            stereoPairs[i]->rightFilename = rightDir + "/" + matchingPairs[i] + ".png";
            
            // Check if files actually exist and update paths if needed
            for (const auto& ext : {".png", ".jpg", ".jpeg"}) {
                std::string leftPath = leftDir + "/" + matchingPairs[i] + ext;
                std::string rightPath = rightDir + "/" + matchingPairs[i] + ext;
                if (fs::exists(leftPath) && fs::exists(rightPath)) {
                    stereoPairs[i]->leftFilename = leftPath;
                    stereoPairs[i]->rightFilename = rightPath;
                    break;
                }
            }
        }
        
        std::cout << "Found " << stereoPairs.size() << " matching stereo pairs" << std::endl;
        
        // Load initial stereo pairs for instant access, then start background loading
        loadInitialStereoPairs();
        startBackgroundLoading();
        
        return true;
    }
    
    SDL_Texture* loadImageTexture(const std::string& filename) {
        SDL_Surface* surface = IMG_Load(filename.c_str());
        if (!surface) {
            std::cerr << "Unable to load image " << filename << "! SDL_image Error: " << IMG_GetError() << std::endl;
            return nullptr;
        }
        
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FreeSurface(surface);
        
        if (!texture) {
            std::cerr << "Unable to create texture from " << filename << "! SDL Error: " << SDL_GetError() << std::endl;
        }
        
        return texture;
    }
    
    void loadInitialStereoPairs() {
        size_t pairCount = stereoPairs.size();
        size_t initialCount = std::min((size_t)INITIAL_LOAD_COUNT, pairCount);
        
        std::cout << "Loading first " << initialCount << " stereo pairs for instant access..." << std::endl;
        
        for (size_t i = 0; i < initialCount; ++i) {
            std::cout << "Loading pair " << (i + 1) << "/" << initialCount << ": " 
                      << stereoPairs[i]->baseName << std::endl;
            
            // Load left surface
            SDL_Surface* leftSurface = IMG_Load(stereoPairs[i]->leftFilename.c_str());
            // Load right surface
            SDL_Surface* rightSurface = IMG_Load(stereoPairs[i]->rightFilename.c_str());
            
            // Apply undistortion if calibration is loaded
            SDL_Surface* undistortedLeftSurface = nullptr;
            SDL_Surface* undistortedRightSurface = nullptr;
            
            if (calibrationLoaded) {
                if (leftSurface) {
                    undistortedLeftSurface = undistortImage(leftSurface, true); // true for left camera
                    SDL_FreeSurface(leftSurface); // Free original distorted surface
                }
                if (rightSurface) {
                    undistortedRightSurface = undistortImage(rightSurface, false); // false for right camera
                    SDL_FreeSurface(rightSurface); // Free original distorted surface
                }
            } else {
                // If no calibration, use original surfaces
                undistortedLeftSurface = leftSurface;
                undistortedRightSurface = rightSurface;
            }
            
            {
                std::lock_guard<std::mutex> lock(imagesMutex);
                
                if (undistortedLeftSurface) {
                    stereoPairs[i]->leftSurface = undistortedLeftSurface;
                    stereoPairs[i]->leftSurfaceLoaded = true;
                    
                    // Create texture immediately for initial pairs
                    stereoPairs[i]->leftTexture = SDL_CreateTextureFromSurface(renderer, undistortedLeftSurface);
                    if (stereoPairs[i]->leftTexture) {
                        stereoPairs[i]->leftTextureCreated = true;
                    }
                }
                
                if (undistortedRightSurface) {
                    stereoPairs[i]->rightSurface = undistortedRightSurface;
                    stereoPairs[i]->rightSurfaceLoaded = true;
                    
                    // Create texture immediately for initial pairs
                    stereoPairs[i]->rightTexture = SDL_CreateTextureFromSurface(renderer, undistortedRightSurface);
                    if (stereoPairs[i]->rightTexture) {
                        stereoPairs[i]->rightTextureCreated = true;
                    }
                }
            }
        }
        
        // Set the next pair index for background loading
        nextImageToLoad = initialCount;
        
        std::cout << "Initial " << initialCount << " stereo pairs loaded! Starting background loading..." << std::endl;
    }
    
    void loadStereoPairInBackground(size_t index) {
        if (index >= stereoPairs.size()) return;
        
        // Load surfaces (this is thread-safe)
        SDL_Surface* leftSurface = IMG_Load(stereoPairs[index]->leftFilename.c_str());
        SDL_Surface* rightSurface = IMG_Load(stereoPairs[index]->rightFilename.c_str());
        
        // Apply undistortion if calibration is loaded
        SDL_Surface* undistortedLeftSurface = nullptr;
        SDL_Surface* undistortedRightSurface = nullptr;
        
        if (calibrationLoaded) {
            if (leftSurface) {
                undistortedLeftSurface = undistortImage(leftSurface, true); // true for left camera
                SDL_FreeSurface(leftSurface); // Free original distorted surface
            }
            if (rightSurface) {
                undistortedRightSurface = undistortImage(rightSurface, false); // false for right camera
                SDL_FreeSurface(rightSurface); // Free original distorted surface
            }
        } else {
            // If no calibration, use original surfaces
            undistortedLeftSurface = leftSurface;
            undistortedRightSurface = rightSurface;
        }
        
        {
            std::lock_guard<std::mutex> lock(imagesMutex);
            
            if (undistortedLeftSurface) {
                stereoPairs[index]->leftSurface = undistortedLeftSurface;
                stereoPairs[index]->leftSurfaceLoaded = true;
            }
            
            if (undistortedRightSurface) {
                stereoPairs[index]->rightSurface = undistortedRightSurface;
                stereoPairs[index]->rightSurfaceLoaded = true;
            }
        }
    }
    
    void ensureStereoTexturesCreated(size_t index) {
        if (index >= stereoPairs.size()) return;
        
        std::lock_guard<std::mutex> lock(imagesMutex);
        
        // Create left texture if surface is loaded but texture not created
        if (stereoPairs[index]->leftSurfaceLoaded && !stereoPairs[index]->leftTextureCreated && stereoPairs[index]->leftSurface) {
            stereoPairs[index]->leftTexture = SDL_CreateTextureFromSurface(renderer, stereoPairs[index]->leftSurface);
            if (stereoPairs[index]->leftTexture) {
                stereoPairs[index]->leftTextureCreated = true;
            }
        }
        
        // Create right texture if surface is loaded but texture not created
        if (stereoPairs[index]->rightSurfaceLoaded && !stereoPairs[index]->rightTextureCreated && stereoPairs[index]->rightSurface) {
            stereoPairs[index]->rightTexture = SDL_CreateTextureFromSurface(renderer, stereoPairs[index]->rightSurface);
            if (stereoPairs[index]->rightTexture) {
                stereoPairs[index]->rightTextureCreated = true;
            }
        }
    }
    
    void backgroundLoadingFunction() {
        size_t pairCount = stereoPairs.size();
        
        while (running) {
            size_t currentIndex = nextImageToLoad.fetch_add(1);
            
            if (currentIndex >= pairCount) {
                break; // All pairs processed
            }
            
            if (currentIndex % 50 == 0 && currentIndex >= static_cast<size_t>(INITIAL_LOAD_COUNT)) {
                std::cout << "Background loading: " << currentIndex << "/" << pairCount << " stereo pairs loaded" << std::endl;
            }
            
            loadStereoPairInBackground(currentIndex);
            
            // Small delay to not overwhelm the system
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        // Check if this was the last thread to finish
        static std::atomic<int> threadsCompleted{0};
        int completed = threadsCompleted.fetch_add(1) + 1;
        if (completed == NUM_LOADING_THREADS) {
            backgroundLoadingComplete = true;
            std::cout << "Background loading complete! All " << pairCount << " stereo pairs loaded." << std::endl;
        }
    }
    
    void startBackgroundLoading() {
        if (stereoPairs.size() > static_cast<size_t>(INITIAL_LOAD_COUNT)) {
            // Start multiple background threads for faster loading
            for (int i = 0; i < NUM_LOADING_THREADS; ++i) {
                backgroundLoaders.emplace_back(&StereoFisheyeViewer::backgroundLoadingFunction, this);
            }
        } else {
            backgroundLoadingComplete = true;
        }
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        
        if (currentIndex >= 0 && currentIndex < static_cast<int>(stereoPairs.size())) {
            // Try to create textures from surfaces if available (main thread only)
            ensureStereoTexturesCreated(currentIndex);
            
            std::lock_guard<std::mutex> lock(imagesMutex);
            
            // Calculate half-window width for side-by-side display
            int halfWidth = windowWidth / 2;
            
            // Render left eye image
            if (stereoPairs[currentIndex]->leftTextureCreated && stereoPairs[currentIndex]->leftTexture) {
                renderEyeImage(stereoPairs[currentIndex]->leftTexture, 0, halfWidth);
            } else {
                renderLoadingMessage(0, halfWidth);
            }
            
            // Render right eye image
            if (stereoPairs[currentIndex]->rightTextureCreated && stereoPairs[currentIndex]->rightTexture) {
                renderEyeImage(stereoPairs[currentIndex]->rightTexture, halfWidth, halfWidth);
            } else {
                renderLoadingMessage(halfWidth, halfWidth);
            }
            
            // Draw divider line between left and right
            SDL_SetRenderDrawColor(renderer, 128, 128, 128, 255);
            SDL_RenderDrawLine(renderer, halfWidth, 0, halfWidth, windowHeight);
        }
        
        SDL_RenderPresent(renderer);
    }
    
    void renderEyeImage(SDL_Texture* texture, int xOffset, int availableWidth) {
        if (!texture) return;
        
        // Get texture dimensions
        int textureWidth, textureHeight;
        SDL_QueryTexture(texture, nullptr, nullptr, &textureWidth, &textureHeight);
        
        // Calculate scaling to fit half window while maintaining aspect ratio
        float scaleX = static_cast<float>(availableWidth) / textureWidth;
        float scaleY = static_cast<float>(windowHeight) / textureHeight;
        float scale = std::min(scaleX, scaleY);
        
        int scaledWidth = static_cast<int>(textureWidth * scale);
        int scaledHeight = static_cast<int>(textureHeight * scale);
        
        SDL_Rect destRect = {
            xOffset + (availableWidth - scaledWidth) / 2,
            (windowHeight - scaledHeight) / 2,
            scaledWidth,
            scaledHeight
        };
        
        SDL_RenderCopy(renderer, texture, nullptr, &destRect);
    }
    
    void renderLoadingMessage(int xOffset, int availableWidth) {
        // Simple loading indicator - draw a white rectangle in the center of the available area
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect loadingRect = {
            xOffset + availableWidth / 2 - 100,
            windowHeight / 2 - 25,
            200,
            50
        };
        SDL_RenderFillRect(renderer, &loadingRect);
        
        // Draw border
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderDrawRect(renderer, &loadingRect);
        
        // Note: For simplicity, we're just showing a white rectangle
        // In a full implementation, you'd use SDL_ttf to render actual text
    }
    
    void handleEvent(SDL_Event& e) {
        if (e.type == SDL_QUIT) {
            running = false;
        } else if (e.type == SDL_KEYDOWN) {
            switch (e.key.keysym.sym) {
                case SDLK_LEFT:
                    previousImage();
                    break;
                case SDLK_RIGHT:
                    nextImage();
                    break;
                case SDLK_ESCAPE:
                    running = false;
                    break;
            }
        } else if (e.window.event == SDL_WINDOWEVENT_RESIZED) {
            windowWidth = e.window.data1;
            windowHeight = e.window.data2;
        }
    }
    
    void nextImage() {
        if (currentIndex < static_cast<int>(stereoPairs.size()) - 1) {
            currentIndex++;
        }
    }
    
    void previousImage() {
        if (currentIndex > 0) {
            currentIndex--;
        }
    }
    
    void run() {
        SDL_Event e;
        
        while (running) {
            while (SDL_PollEvent(&e)) {
                handleEvent(e);
            }
            
            render();
            SDL_Delay(16); // ~60 FPS
        }
    }
    
    void cleanup() {
        running = false;
        
        // Wait for all background loading threads to finish
        for (auto& loader : backgroundLoaders) {
            if (loader.joinable()) {
                loader.join();
            }
        }
        backgroundLoaders.clear();
        
        stereoPairs.clear();
        
        if (renderer) {
            SDL_DestroyRenderer(renderer);
            renderer = nullptr;
        }
        
        if (window) {
            SDL_DestroyWindow(window);
            window = nullptr;
        }
        
        IMG_Quit();
        SDL_Quit();
    }
};

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <left_directory> <right_directory>" << std::endl;
        std::cerr << "Example: " << argv[0] << " /path/to/left/images /path/to/right/images" << std::endl;
        return 1;
    }
    
    std::string leftDirectory = argv[1];
    std::string rightDirectory = argv[2];
    
    if (!fs::exists(leftDirectory) || !fs::is_directory(leftDirectory)) {
        std::cerr << "Error: " << leftDirectory << " is not a valid directory" << std::endl;
        return 1;
    }
    
    if (!fs::exists(rightDirectory) || !fs::is_directory(rightDirectory)) {
        std::cerr << "Error: " << rightDirectory << " is not a valid directory" << std::endl;
        return 1;
    }
    
    StereoFisheyeViewer viewer;
    
    if (!viewer.initialize()) {
        std::cerr << "Failed to initialize SDL" << std::endl;
        return 1;
    }
    
    // Load fisheye calibration for undistortion
    if (!viewer.loadCalibration()) {
        std::cerr << "Warning: Failed to load calibration data. Images will be displayed without undistortion." << std::endl;
    }
    
    if (!viewer.loadStereoPairs(leftDirectory, rightDirectory)) {
        std::cerr << "Failed to load stereo pairs from directories" << std::endl;
        return 1;
    }
    
    std::cout << "Use left/right arrow keys to navigate unwrapped stereo pairs, ESC to quit" << std::endl;
    std::cout << "Left half: image_02 (unwrapped), Right half: image_03 (unwrapped)" << std::endl;
    viewer.run();
    
    return 0;
}