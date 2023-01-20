#include "renderer.h"

#include <variant>

#include "image.h"

uint32_t utils::colorToRGBA(const color& color) {
    uint8_t r = (uint8_t)(color.r * 255.f);
    uint8_t g = (uint8_t)(color.g * 255.f);
    uint8_t b = (uint8_t)(color.b * 255.f);
    uint8_t a = (uint8_t)(color.a * 255.f);
    uint32_t rgba_val = (a << 24) | (b << 16) | (g << 8) | r;
    return rgba_val;
}

color ObjectRenderer::operator()(const Plane& plane) const {
    // return plane.color;
    return m_background_color;
}

color ObjectRenderer::operator()(const Cube& cube) const {
    // return cube.color;
    return m_background_color;
}

color ObjectRenderer::operator()(const Sphere& sphere) const {
    const vec3 sphere_center_to_ray_orig = m_ray.orig - sphere.position;
    const auto a = dot(m_ray.dir, m_ray.dir);
    const auto b = 2.f * dot(sphere_center_to_ray_orig, m_ray.dir);
    const auto c = dot(sphere_center_to_ray_orig, sphere_center_to_ray_orig) - sphere.radius * sphere.radius;
    const auto discriminant = b * b - 4 * a * c;
    if (discriminant > 0) {  // two roots = hit
        return sphere.color;
    }
    return m_background_color;
}

Renderer::Renderer(const Camera& camera) : m_camera(camera), m_background_color(0.1f, 0.1f, 0.1f, 1.f) {}

void Renderer::refresh(uint32_t width, uint32_t height) {
    if (m_image) {
        if (m_image->width() != width || m_image->height() != height) {
            m_image->resize(width, height);
        }
    } else {
        m_image = std::make_shared<Image>(width, height);
    }
    delete[] m_img_data;
    m_img_data = new uint32_t[width * height];
}

color Renderer::getPixelColor(const Scene& scene, uint32_t x, uint32_t y) {
    Ray ray(m_camera.position(), m_camera.rayDirection(x, y));
    for (const auto& object : scene.objects()) {
        return std::visit(ObjectRenderer(ray), *object);  // TODO: objects need to be sorted, return for first in depth
    }
    return m_background_color;
}

void Renderer::render(const Scene& scene) {
    for (uint32_t y = 0; y < m_image->height(); ++y) {
        for (uint32_t x = 0; x < m_image->width(); ++x) {
            const auto color = getPixelColor(scene, x, y);
            m_img_data[x + y * m_image->width()] = utils::colorToRGBA(color);
        }
    }
    m_image->setData(m_img_data);
}
