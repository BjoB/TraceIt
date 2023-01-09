#include "renderer.h"

#include "image.h"

uint32_t utils::colorToRGBA(const glm::vec4& color) {
    uint8_t r = (uint8_t)(color.r * 255.f);
    uint8_t g = (uint8_t)(color.g * 255.f);
    uint8_t b = (uint8_t)(color.b * 255.f);
    uint8_t a = (uint8_t)(color.a * 255.f);
    uint32_t rgba_val = (a << 24) | (b << 16) | (g << 8) | r;
    return rgba_val;
}

void Renderer::refresh(uint32_t width, uint32_t height) {
    if (m_img_data) {
        if (m_image->width() != width || m_image->height() != height) {
            // TODO: resize Image
        }
    } else {
        delete[] m_img_data;
        m_img_data = new uint32_t[width * height];
        m_image = std::make_shared<Image>(width, height, m_img_data);
    }
}

void Renderer::render(const Scene& scene) {
    for (uint32_t i = 0; i < sizeof(m_img_data); ++i) {
        m_img_data[i] = utils::colorToRGBA(glm::vec4(0.2f));
    }
    m_image->setData(m_img_data);
    //for (auto& object : scene.objects()) {
    //}
}
