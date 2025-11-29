#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
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
    
    // Background loading
    std::vector<std::thread> backgroundLoaders;
    std::mutex imagesMutex;
    std::atomic<bool> backgroundLoadingComplete;
    std::atomic<size_t> nextImageToLoad;
    const int INITIAL_LOAD_COUNT = 10;
    const int NUM_LOADING_THREADS = 4;
    
public:
    StereoFisheyeViewer() : window(nullptr), renderer(nullptr), currentIndex(0), 
                            windowWidth(1600), windowHeight(800), running(true), 
                            backgroundLoadingComplete(false), nextImageToLoad(0) {}
    
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
        
        window = SDL_CreateWindow("Dual Fisheye Camera Viewer - Stereo",
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
            
            {
                std::lock_guard<std::mutex> lock(imagesMutex);
                
                if (leftSurface) {
                    stereoPairs[i]->leftSurface = leftSurface;
                    stereoPairs[i]->leftSurfaceLoaded = true;
                    
                    // Create texture immediately for initial pairs
                    stereoPairs[i]->leftTexture = SDL_CreateTextureFromSurface(renderer, leftSurface);
                    if (stereoPairs[i]->leftTexture) {
                        stereoPairs[i]->leftTextureCreated = true;
                    }
                }
                
                if (rightSurface) {
                    stereoPairs[i]->rightSurface = rightSurface;
                    stereoPairs[i]->rightSurfaceLoaded = true;
                    
                    // Create texture immediately for initial pairs
                    stereoPairs[i]->rightTexture = SDL_CreateTextureFromSurface(renderer, rightSurface);
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
        
        {
            std::lock_guard<std::mutex> lock(imagesMutex);
            
            if (leftSurface) {
                stereoPairs[index]->leftSurface = leftSurface;
                stereoPairs[index]->leftSurfaceLoaded = true;
            }
            
            if (rightSurface) {
                stereoPairs[index]->rightSurface = rightSurface;
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
    
    if (!viewer.loadStereoPairs(leftDirectory, rightDirectory)) {
        std::cerr << "Failed to load stereo pairs from directories" << std::endl;
        return 1;
    }
    
    std::cout << "Use left/right arrow keys to navigate stereo pairs, ESC to quit" << std::endl;
    viewer.run();
    
    return 0;
}