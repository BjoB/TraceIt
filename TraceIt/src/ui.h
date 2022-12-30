#pragma once

#define GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vulkan/vulkan.h>

namespace Ui {

struct Config {
    const char* name;
    int window_width;
    int window_height;
};

class App {
   public:
    App(const Config& config);
    App(const App&) = delete;
    App& operator=(const App) = delete;
    virtual ~App();

    void run();
    GLFWwindow* getWnd() { return m_window; }

   private:
    void init(const char* name, int window_width, int window_height);
    void cleanup();

    GLFWwindow* m_window = nullptr;
    bool m_running = false;
};

}  // namespace Ui
