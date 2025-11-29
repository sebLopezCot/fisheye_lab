#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <filesystem>
#include <thread>
#include <mutex>
#include <queue>
#include <memory>

namespace fs = std::filesystem;

struct ImageData {
    SDL_Texture* texture;
    SDL_Surface* surface;
    std::string filename;
    bool loaded;
    bool surfaceLoaded;
    
    ImageData() : texture(nullptr), surface(nullptr), loaded(false), surfaceLoaded(false) {}
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
    std::thread prefetchThread;
    std::mutex imageMutex;
    std::queue<int> prefetchQueue;
    bool running;
    
    const int PREFETCH_COUNT = 20; // Number of images to prefetch ahead/behind
    
public:
    FisheyeViewer() : window(nullptr), renderer(nullptr), currentIndex(0), 
                      windowWidth(1280), windowHeight(720), running(true) {}
    
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
        
        // Initialize image data structures
        images.resize(imageFiles.size());
        for (size_t i = 0; i < images.size(); ++i) {
            images[i] = std::make_unique<ImageData>();
            images[i]->filename = imageFiles[i];
        }
        
        std::cout << "Loaded " << imageFiles.size() << " image files" << std::endl;
        
        // Load the first image surface immediately
        loadImageSurface(0);
        
        // Start prefetching thread
        prefetchThread = std::thread(&FisheyeViewer::prefetchImages, this);
        
        return true;
    }
    
    
    SDL_Texture* createTextureFromSurface(SDL_Surface* surface) {
        if (!surface) return nullptr;
        
        SDL_Texture* texture = SDL_CreateTextureFromSurface(renderer, surface);
        if (!texture) {
            std::cerr << "Unable to create texture! SDL Error: " << SDL_GetError() << std::endl;
        }
        return texture;
    }
    
    void loadImageSurface(int index) {
        if (index < 0 || index >= static_cast<int>(images.size())) return;
        
        std::lock_guard<std::mutex> lock(imageMutex);
        
        if (!images[index]->surfaceLoaded) {
            images[index]->surface = IMG_Load(images[index]->filename.c_str());
            if (images[index]->surface) {
                images[index]->surfaceLoaded = true;
            } else {
                std::cerr << "Unable to load image " << images[index]->filename << "! SDL_image Error: " << IMG_GetError() << std::endl;
            }
        }
    }
    
    void ensureTextureLoaded(int index) {
        if (index < 0 || index >= static_cast<int>(images.size())) return;
        
        std::lock_guard<std::mutex> lock(imageMutex);
        
        if (!images[index]->loaded && images[index]->surfaceLoaded && images[index]->surface) {
            images[index]->texture = createTextureFromSurface(images[index]->surface);
            if (images[index]->texture) {
                images[index]->loaded = true;
                // Free surface after creating texture
                SDL_FreeSurface(images[index]->surface);
                images[index]->surface = nullptr;
            }
        }
    }
    
    void prefetchImages() {
        while (running) {
            std::vector<int> indicesToLoad;
            
            // Determine which surfaces to prefetch around current index
            {
                std::lock_guard<std::mutex> lock(imageMutex);
                for (int offset = -PREFETCH_COUNT; offset <= PREFETCH_COUNT; ++offset) {
                    int index = currentIndex + offset;
                    if (index >= 0 && index < static_cast<int>(images.size()) && !images[index]->surfaceLoaded) {
                        indicesToLoad.push_back(index);
                    }
                }
            }
            
            // Load surfaces (safe to do from background thread)
            for (int index : indicesToLoad) {
                if (!running) break;
                loadImageSurface(index);
            }
            
            SDL_Delay(100); // Small delay to prevent busy waiting
        }
    }
    
    void render() {
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        
        if (currentIndex >= 0 && currentIndex < static_cast<int>(images.size())) {
            // Ensure current image texture is created (must be done on main thread)
            ensureTextureLoaded(currentIndex);
            
            std::lock_guard<std::mutex> lock(imageMutex);
            
            if (images[currentIndex]->loaded && images[currentIndex]->texture) {
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
            }
        }
        
        SDL_RenderPresent(renderer);
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
            loadImageSurface(currentIndex); // Load surface from any thread
        }
    }
    
    void previousImage() {
        if (currentIndex > 0) {
            currentIndex--;
            loadImageSurface(currentIndex); // Load surface from any thread
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
        
        if (prefetchThread.joinable()) {
            prefetchThread.join();
        }
        
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