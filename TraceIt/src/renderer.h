#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>

#include "image.h"
#include "scene.h"

namespace utils {

uint32_t colorToRGBA(const glm::vec4& color);

}

struct ObjectRenderer {
    void operator()(const Plane& plane) const;
    void operator()(const Cube& cube) const;
    void operator()(const Sphere& sphere) const;
};

class Renderer {
   public:
    Renderer() = default;

    void refresh(uint32_t width, uint32_t height);
    void render(const Scene& scene);

    std::shared_ptr<Image> image() const { return m_image; }

   private:
    void render(const SceneObject& object);
    void render(const Plane& plane);
    void render(const Cube& cube);
    void render(const Sphere& sphere);

    uint32_t* m_img_data = nullptr;
    std::shared_ptr<Image> m_image;
};
