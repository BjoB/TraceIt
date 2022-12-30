#pragma once

#define GLFW_INCLUDE_NONE
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vulkan/vulkan.h>

#include <memory>
#include <type_traits>
#include <vector>

namespace Ui {

class Layer {
   public:
    virtual ~Layer() = default;

    virtual void onUpdate() {}
    virtual void onRender() {}
};

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
    void stop();

    template <class DerivedLayerT>
    void addLayer() {
        static_assert(std::is_base_of_v<Layer, DerivedLayerT>, "UI layers need to be derived from Layer base class!");
        m_layers.push_back(std::make_shared<DerivedLayerT>());
    }

    GLFWwindow* getWnd() const { return m_window; }

   private:
    void init(const char* name, int window_width, int window_height);
    void cleanup();
    void updateFrameTime();
    float currentTimeMs() const { return static_cast<float>(1e-3 * glfwGetTime()); }

    bool m_running = false;
    float m_frame_duration = 0.f;
    float m_time_last_frame = 0.f;
    GLFWwindow* m_window = nullptr;
    std::vector<std::shared_ptr<Layer>> m_layers;
};

}  // namespace Ui
