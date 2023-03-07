#include "renderer.h"

#include <variant>

uint32_t utils::colorToRGBA(const color& color) {
    uint8_t r = (uint8_t)(color.r * 255.f);
    uint8_t g = (uint8_t)(color.g * 255.f);
    uint8_t b = (uint8_t)(color.b * 255.f);
    uint8_t a = (uint8_t)(color.a * 255.f);
    uint32_t rgba_val = (a << 24) | (b << 16) | (g << 8) | r;
    return rgba_val;
}

Renderer::Renderer(const Camera& camera) : m_camera(camera), m_background_color(0.1f, 0.1f, 0.1f, 1.f) {}

void Renderer::refresh(uint32_t width, uint32_t height) {
#if !defined(WITHOUT_VULKAN_IMG)
    if (m_image) {
        if (m_image->width() != width || m_image->height() != height) {
            m_image->resize(width, height);
        }
    } else {
        m_image = std::make_shared<Image>(width, height);
    }
#else
    m_image_width = width;
    m_image_height = height;
#endif
    delete[] m_img_data;
    m_img_data = new uint32_t[width * height];
}

color Renderer::getPixelColor(const Scene& scene, uint32_t x, uint32_t y) {
    Ray ray(m_camera.position(), m_camera.rayDirection(x, y));
    RayHitRecord rec;
    if (scene.hit(ray, 0.f, m_t_max, rec)) {
        return rec.color;
    }
    return m_background_color;
}

void Renderer::render(Scene& scene) {
#if !defined(WITHOUT_VULKAN_IMG)
    const auto img_width = m_image->width();
    const auto img_height = m_image->height();
#else
    const auto img_width = m_image_width;
    const auto img_height = m_image_height;
#endif
    for (uint32_t y = 0; y < img_height; ++y) {
        for (uint32_t x = 0; x < img_width; ++x) {
            const auto color = getPixelColor(scene, x, y);
            m_img_data[x + y * img_width] = utils::colorToRGBA(color);
        }
    }

#if !defined(WITHOUT_VULKAN_IMG)
    m_image->setData(m_img_data);
#endif

    scene.cleanUp();
}
