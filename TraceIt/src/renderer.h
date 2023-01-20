#pragma once

#include <glm/glm.hpp>

#include "image.h"
#include "scene.h"
#include "camera.h"

using namespace glm;
using color = vec4;

namespace utils {

uint32_t colorToRGBA(const color& color);

}

class Ray {
   public:
    using Scalar = vec3::value_type;

    Ray(const vec3& origin, const vec3& direction) : orig(origin), dir(direction) {}

    vec3 at(Scalar t) const { return orig + t * dir; }

   public:
    vec3 orig;
    vec3 dir;
};

struct ObjectRenderer {
    ObjectRenderer(const Ray& ray) : m_ray(ray), m_background_color(0.5f, 0.6f, 1.f, 1.f) {}

    color operator()(const Plane& plane) const;
    color operator()(const Cube& cube) const;
    color operator()(const Sphere& sphere) const;

    Ray m_ray;
    color m_background_color;
};

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
    void render(const SceneObject& object);
    void render(const Plane& plane);
    void render(const Cube& cube);
    void render(const Sphere& sphere);

    uint32_t* m_img_data = nullptr;
    std::shared_ptr<Image> m_image;
    const Camera& m_camera;
    color m_background_color;
};
