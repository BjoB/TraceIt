#include "renderer.h"

#include <variant>

#include "image.h"

uint32_t utils::colorToRGBA(const glm::vec4& color) {
    uint8_t r = (uint8_t)(color.r * 255.f);
    uint8_t g = (uint8_t)(color.g * 255.f);
    uint8_t b = (uint8_t)(color.b * 255.f);
    uint8_t a = (uint8_t)(color.a * 255.f);
    uint32_t rgba_val = (a << 24) | (b << 16) | (g << 8) | r;
    return rgba_val;
}

Renderer::Renderer(const Camera& camera) : m_camera(camera) {}

void ObjectRenderer::operator()(const Plane& plane) const {}

void ObjectRenderer::operator()(const Cube& cube) const {}

void ObjectRenderer::operator()(const Sphere& sphere) const {}

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

vec4 Renderer::getPixelColor(const Scene& scene, uint32_t x, uint32_t y) {
    // TODO: add logic
    for (const auto& object : scene.objects()) {
        std::visit(ObjectRenderer{}, *object);
    }
    return vec4(0.2);
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
