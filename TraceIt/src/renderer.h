#pragma once

#include <glm/glm.hpp>

#include "camera.h"
#include "image.h"
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
    void render(const Scene& scene);

    std::shared_ptr<Image> image() const { return m_image; }

   private:
    color getPixelColor(const Scene& scene, uint32_t x, uint32_t y);
    void render(const HittableObjectVariant& object);
    void render(const Plane& plane);
    void render(const Cube& cube);
    void render(const Sphere& sphere);

   private:
    const float m_t_max = 100.f;
    uint32_t* m_img_data = nullptr;
    std::shared_ptr<Image> m_image;
    const Camera& m_camera;
    color m_background_color;
};
