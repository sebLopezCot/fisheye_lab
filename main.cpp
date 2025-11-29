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

namespace fs = std::filesystem;

struct ImageData {
    SDL_Texture* texture;
    SDL_Surface* surface;
    std::string filename;
    std::atomic<bool> surfaceLoaded;
    std::atomic<bool> textureCreated;
    
    ImageData() : texture(nullptr), surface(nullptr), surfaceLoaded(false), textureCreated(false) {}
    ~ImageData() {
        if (texture) {
            SDL_DestroyTexture(texture);
        }
        if (surface) {
            SDL_FreeSurface(surface);
        }
    }
};

class FisheyeViewer {
private:
    SDL_Window* window;
    SDL_Renderer* renderer;
    std::vector<std::string> imageFiles;
    std::vector<std::unique_ptr<ImageData>> images;
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
    FisheyeViewer() : window(nullptr), renderer(nullptr), currentIndex(0), 
                      windowWidth(1280), windowHeight(720), running(true), 
                      backgroundLoadingComplete(false), nextImageToLoad(0) {}
    
    ~FisheyeViewer() {
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
        
        window = SDL_CreateWindow("Fisheye Camera Viewer",
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
    
    bool loadImageList(const std::string& directory) {
        try {
            for (const auto& entry : fs::directory_iterator(directory)) {
                if (entry.is_regular_file()) {
                    std::string extension = entry.path().extension().string();
                    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);
                    
                    if (extension == ".jpg" || extension == ".jpeg" || extension == ".png") {
                        imageFiles.push_back(entry.path().string());
                    }
                }
            }
        } catch (const std::exception& e) {
            std::cerr << "Error reading directory: " << e.what() << std::endl;
            return false;
        }
        
        if (imageFiles.empty()) {
            std::cerr << "No image files found in directory: " << directory << std::endl;
            return false;
        }
        
        // Sort files in ascending order
        std::sort(imageFiles.begin(), imageFiles.end());
        
        // Check if we have too many images and ask user
        if (imageFiles.size() > 2000) {
            std::cout << "\nFound " << imageFiles.size() << " images. This is a large dataset." << std::endl;
            std::cout << "Loading all images may use significant memory and time." << std::endl;
            std::cout << "Do you want to:" << std::endl;
            std::cout << "  1. Load all " << imageFiles.size() << " images" << std::endl;
            std::cout << "  2. Load only the first 2000 images" << std::endl;
            std::cout << "Enter your choice (1 or 2): ";
            
            std::string choice;
            std::getline(std::cin, choice);
            
            if (choice == "2") {
                imageFiles.resize(2000);
                std::cout << "Limiting to first 2000 images." << std::endl;
            } else {
                std::cout << "Loading all " << imageFiles.size() << " images." << std::endl;
            }
        }
        
        // Initialize image data structures
        images.resize(imageFiles.size());
        for (size_t i = 0; i < images.size(); ++i) {
            images[i] = std::make_unique<ImageData>();
            images[i]->filename = imageFiles[i];
        }
        
        std::cout << "Found " << imageFiles.size() << " image files" << std::endl;
        
        // Load initial images for instant access, then start background loading
        loadInitialImages();
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
    
    void loadInitialImages() {
        size_t imageCount = images.size();
        size_t initialCount = std::min((size_t)INITIAL_LOAD_COUNT, imageCount);
        
        std::cout << "Loading first " << initialCount << " images for instant access..." << std::endl;
        
        for (size_t i = 0; i < initialCount; ++i) {
            std::cout << "Loading image " << (i + 1) << "/" << initialCount << ": " 
                      << fs::path(images[i]->filename).filename().string() << std::endl;
            
            // Load surface first
            SDL_Surface* surface = IMG_Load(images[i]->filename.c_str());
            if (surface) {
                std::lock_guard<std::mutex> lock(imagesMutex);
                images[i]->surface = surface;
                images[i]->surfaceLoaded = true;
                
                // Create texture immediately for initial images
                images[i]->texture = SDL_CreateTextureFromSurface(renderer, surface);
                if (images[i]->texture) {
                    images[i]->textureCreated = true;
                }
            }
        }
        
        // Set the next image index for background loading
        nextImageToLoad = initialCount;
        
        std::cout << "Initial " << initialCount << " images loaded! Starting background loading..." << std::endl;
    }
    
    void loadSurfaceInBackground(size_t index) {
        if (index >= images.size()) return;
        
        // Load surface (this is thread-safe)
        SDL_Surface* surface = IMG_Load(images[index]->filename.c_str());
        
        if (surface) {
            std::lock_guard<std::mutex> lock(imagesMutex);
            images[index]->surface = surface;
            images[index]->surfaceLoaded = true;
        }
    }
    
    void ensureTextureCreated(size_t index) {
        if (index >= images.size()) return;
        
        std::lock_guard<std::mutex> lock(imagesMutex);
        
        // If surface is loaded but texture not created, create it now
        if (images[index]->surfaceLoaded && !images[index]->textureCreated && images[index]->surface) {
            images[index]->texture = SDL_CreateTextureFromSurface(renderer, images[index]->surface);
            if (images[index]->texture) {
                images[index]->textureCreated = true;
                // Keep the surface for potential future use, or free it to save memory
                // SDL_FreeSurface(images[index]->surface);
                // images[index]->surface = nullptr;
            }
        }
    }
    
    void backgroundLoadingFunction() {
        size_t imageCount = images.size();
        
        while (running) {
            size_t currentIndex = nextImageToLoad.fetch_add(1);
            
            if (currentIndex >= imageCount) {
                break; // All images processed
            }
            
            if (currentIndex % 50 == 0 && currentIndex >= static_cast<size_t>(INITIAL_LOAD_COUNT)) {
                std::cout << "Background loading: " << currentIndex << "/" << imageCount << " surfaces loaded" << std::endl;
            }
            
            loadSurfaceInBackground(currentIndex);
            
            // Small delay to not overwhelm the system
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        
        // Check if this was the last thread to finish
        static std::atomic<int> threadsCompleted{0};
        int completed = threadsCompleted.fetch_add(1) + 1;
        if (completed == NUM_LOADING_THREADS) {
            backgroundLoadingComplete = true;
            std::cout << "Background surface loading complete! All " << imageCount << " surfaces loaded." << std::endl;
        }
    }
    
    void startBackgroundLoading() {
        if (images.size() > static_cast<size_t>(INITIAL_LOAD_COUNT)) {
            // Start multiple background threads for faster loading
            for (int i = 0; i < NUM_LOADING_THREADS; ++i) {
                backgroundLoaders.emplace_back(&FisheyeViewer::backgroundLoadingFunction, this);
            }
        } else {
            backgroundLoadingComplete = true;
        }
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        
        if (currentIndex >= 0 && currentIndex < static_cast<int>(images.size())) {
            // Try to create texture from surface if available (main thread only)
            ensureTextureCreated(currentIndex);
            
            std::lock_guard<std::mutex> lock(imagesMutex);
            
            if (images[currentIndex]->textureCreated && images[currentIndex]->texture) {
                // Get texture dimensions
                int textureWidth, textureHeight;
                SDL_QueryTexture(images[currentIndex]->texture, nullptr, nullptr, &textureWidth, &textureHeight);
                
                // Calculate scaling to fit window while maintaining aspect ratio
                float scaleX = static_cast<float>(windowWidth) / textureWidth;
                float scaleY = static_cast<float>(windowHeight) / textureHeight;
                float scale = std::min(scaleX, scaleY);
                
                int scaledWidth = static_cast<int>(textureWidth * scale);
                int scaledHeight = static_cast<int>(textureHeight * scale);
                
                SDL_Rect destRect = {
                    (windowWidth - scaledWidth) / 2,
                    (windowHeight - scaledHeight) / 2,
                    scaledWidth,
                    scaledHeight
                };
                
                SDL_RenderCopy(renderer, images[currentIndex]->texture, nullptr, &destRect);
            } else {
                // Display loading message for unloaded images
                renderLoadingMessage();
            }
        }
        
        SDL_RenderPresent(renderer);
    }
    
    void renderLoadingMessage() {
        // Simple loading indicator - draw a white rectangle in the center
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect loadingRect = {
            windowWidth / 2 - 100,
            windowHeight / 2 - 25,
            200,
            50
        };
        SDL_RenderFillRect(renderer, &loadingRect);
        
        // Draw border
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderDrawRect(renderer, &loadingRect);
        
        // Note: For simplicity, we're just showing a white rectangle
        // In a full implementation, you'd use SDL_ttf for actual text
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
        } else if (e.type == SDL_WINDOWEVENT) {
            if (e.window.event == SDL_WINDOWEVENT_RESIZED) {
                windowWidth = e.window.data1;
                windowHeight = e.window.data2;
            }
        }
    }
    
    void nextImage() {
        if (currentIndex < static_cast<int>(images.size()) - 1) {
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
        
        images.clear();
        
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
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <image_directory>" << std::endl;
        return 1;
    }
    
    std::string imageDirectory = argv[1];
    
    if (!fs::exists(imageDirectory) || !fs::is_directory(imageDirectory)) {
        std::cerr << "Error: " << imageDirectory << " is not a valid directory" << std::endl;
        return 1;
    }
    
    FisheyeViewer viewer;
    
    if (!viewer.initialize()) {
        std::cerr << "Failed to initialize SDL" << std::endl;
        return 1;
    }
    
    if (!viewer.loadImageList(imageDirectory)) {
        std::cerr << "Failed to load images from directory" << std::endl;
        return 1;
    }
    
    std::cout << "Use left/right arrow keys to navigate, ESC to quit" << std::endl;
    viewer.run();
    
    return 0;
}
