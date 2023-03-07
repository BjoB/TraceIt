#pragma once

// With active compile definition "WITHOUT_VULKAN_IMG", we switch to a Vulkan free CLI version of the renderer.

#include <glm/glm.hpp>

#include "camera.h"
#if !defined(WITHOUT_VULKAN_IMG)
#include "image.h"
#endif
#include "scene.h"

using color = glm::vec4;

namespace utils {

uint32_t colorToRGBA(const color& color);

}

class Renderer {
   public:
    Renderer(const Camera& camera);
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    void refresh(uint32_t width, uint32_t height);
    void render(Scene& scene);

#if !defined(WITHOUT_VULKAN_IMG)
    std::shared_ptr<Image> image() const { return m_image; }
#endif

    uint32_t* imageData() const { return m_img_data; }

   private:
    color getPixelColor(const Scene& scene, uint32_t x, uint32_t y);
    void render(const HittableObjectVariant& object);
    void render(const Plane& plane);
    void render(const Cube& cube);
    void render(const Sphere& sphere);

   private:
    const float m_t_max = 100.f;
    uint32_t* m_img_data = nullptr;
#if !defined(WITHOUT_VULKAN_IMG)
    std::shared_ptr<Image> m_image;
#else
    uint32_t m_image_width = 0;
    uint32_t m_image_height = 0;
#endif
    const Camera& m_camera;
    color m_background_color;
};
