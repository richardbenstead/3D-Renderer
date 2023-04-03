#include <GL/gl.h>
#include <GLFW/glfw3.h>
#include <chrono>
#include <iostream>

#include "canvas.h"
#include "render.h"
#include "sceneClock.h"
#include "sceneCubeArray.h"
#include "sceneTileArray.h"

static constexpr uint16_t IMAGE_SIZE = 1200;

template <int16_t GS> class Image {
  public:
    using colRGB_t = colRGB;
    static constexpr int16_t GRID_SIZE{GS};
    static constexpr int32_t ARR_SIZE{GS * GS};

    constexpr static int32_t POS(int16_t i, int16_t j) { return i + GRID_SIZE * j; };
    std::array<colRGB, ARR_SIZE> image{};
};

class GlWinImage {
    using ImageType = Image<IMAGE_SIZE>;
    auto POS(auto x, auto y) { return ImageType::POS(x, y); }

  public:
    double secondsPerFrame = 0;
    std::chrono::high_resolution_clock::time_point lastLog_{};

    void draw() {
        int width, height;
        glfwGetWindowSize(mpWindow, &width, &height);

        glLoadIdentity();
        glMatrixMode(GL_MODELVIEW);
        glOrtho(0, width, 0, height, -1.0, 1.0);
        glViewport(0, 0, width, height);

        scene_.update();
        auto start = std::chrono::high_resolution_clock::now();
        Render::drawImage(canvas_, scene_);
        auto stop = std::chrono::high_resolution_clock::now();
        std::chrono::microseconds duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        secondsPerFrame = std::lerp(secondsPerFrame, duration.count() / 1e6, 0.05);

        if (std::chrono::duration_cast<std::chrono::seconds>(stop - lastLog_).count() > 5) {
            lastLog_ = stop;
            std::cout << "Seconds per frame: " << secondsPerFrame << std::endl;
        }

        glPixelZoom(width / static_cast<float>(IMAGE_SIZE), -height / static_cast<float>(IMAGE_SIZE));
        glRasterPos2i(0, height);
        glDrawPixels(IMAGE_SIZE, IMAGE_SIZE, GL_RGB, GL_FLOAT, &canvas_._image.image[0]);

        glfwSwapBuffers(mpWindow);
        glfwPollEvents();
    }

    void run() {
        while (!mQuit && !glfwWindowShouldClose(mpWindow)) {
            draw();
        }
    }

    void initialize(const std::string &title) {
        mpWindow = glfwCreateWindow(1200, 800, title.c_str(), nullptr, nullptr);
        if (!mpWindow) {
            throw std::runtime_error("glfwCreateWindow failed");
        }

        glfwMakeContextCurrent(mpWindow);
        glfwSetWindowUserPointer(mpWindow, this);

        glfwSetKeyCallback(mpWindow, [](GLFWwindow *window, int key, int sc, int action, int mods) {
            static_cast<GlWinImage *>(glfwGetWindowUserPointer(window))->keyEvent(window, key, sc, action, mods);
        });
    }

  private:
    void keyEvent([[maybe_unused]] GLFWwindow *window, int key, [[maybe_unused]] int sc, int action,
                  [[maybe_unused]] int mods) {
        if (GLFW_PRESS == action) {
            if (key == 'Q')
                mQuit = true;
            if (key == 'P')
                scene_._pause = !scene_._pause;
            if (key == ' ') {
                scene_.step();
            }
        }
    }

    GLFWwindow *mpWindow{};
    bool mQuit = false;
    Canvas<ImageType> canvas_;
    SceneTileArray scene_;
    // SceneCubeArray scene_;
    // SceneClock scene_;
};

int main(int, char **) {
    // initialize GL
    if (!glfwInit()) {
        throw std::runtime_error("glfwInit failed");
    }

    // Create GL window and repeately update it
    GlWinImage *winImage = new GlWinImage();
    winImage->initialize("3d Render");
    winImage->run();
    delete winImage;
    return 0;
}
