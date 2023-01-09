#pragma once

// #define STB_IMAGE_IMPLEMENTATION
// #include <stb_image.h>

#include <vulkan/vulkan.h>

#include "ui.h"

class Image {
   public:
    Image(uint32_t width, uint32_t height, const void* img_data);
    ~Image();

    void setData(const void* img_data);
    void resize(uint32_t width, uint32_t height);

    auto width() const { return m_width; }
    auto height() const { return m_height; }
    auto descrSet() const { return m_descr_set; }

   private:
    void initialize();
    void cleanup();

    const VkFormat m_image_format = VK_FORMAT_R8G8B8A8_UNORM;
    const uint32_t m_bytes_per_pixel = 4;

    uint32_t m_width;
    uint32_t m_height;
    uint32_t m_channels;

    VkDescriptorSet m_descr_set = nullptr;

    // Need to keep track of these to properly cleanup
    VkImageView m_image_view = nullptr;
    VkImage m_image = nullptr;
    VkDeviceMemory m_image_memory = nullptr;
    VkSampler m_sampler = nullptr;
    VkBuffer m_upload_buffer = nullptr;
    VkDeviceMemory m_upload_buffer_memory = nullptr;
};
