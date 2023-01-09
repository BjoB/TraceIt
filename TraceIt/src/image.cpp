#include "image.h"

#include <vulkan/vulkan.h>

#include <stdexcept>

#include "backends/imgui_impl_vulkan.h"

Image::Image(uint32_t width, uint32_t height, const void* img_data) : m_width(width), m_height(height) {
    initialize();
    if (img_data) {
        setData(img_data);
    }
}

Image::~Image() { cleanup(); }

void Image::initialize() {
    VkResult err;
    auto device = Ui::App::getVkDevice();

    // Create the Vulkan image
    {
        VkImageCreateInfo info = {};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        info.imageType = VK_IMAGE_TYPE_2D;
        info.format = VK_FORMAT_R8G8B8A8_UNORM;
        info.extent.width = m_width;
        info.extent.height = m_height;
        info.extent.depth = 1;
        info.mipLevels = 1;
        info.arrayLayers = 1;
        info.samples = VK_SAMPLE_COUNT_1_BIT;
        info.tiling = VK_IMAGE_TILING_OPTIMAL;
        info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
        info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
        info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        err = vkCreateImage(device, &info, nullptr, &m_image);
        Ui::checkVkResult(err);
        VkMemoryRequirements req;
        vkGetImageMemoryRequirements(device, m_image, &req);
        VkMemoryAllocateInfo alloc_info = {};
        alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        alloc_info.allocationSize = req.size;
        alloc_info.memoryTypeIndex = Ui::findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        err = vkAllocateMemory(device, &alloc_info, nullptr, &m_image_memory);
        Ui::checkVkResult(err);
        err = vkBindImageMemory(device, m_image, m_image_memory, 0);
        Ui::checkVkResult(err);
    }

    // Create the Image View
    {
        VkImageViewCreateInfo info = {};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        info.image = m_image;
        info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        info.format = VK_FORMAT_R8G8B8A8_UNORM;
        info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        info.subresourceRange.levelCount = 1;
        info.subresourceRange.layerCount = 1;
        err = vkCreateImageView(device, &info, nullptr, &m_image_view);
        Ui::checkVkResult(err);
    }

    // Create Sampler
    {
        VkSamplerCreateInfo sampler_info{};
        sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        sampler_info.magFilter = VK_FILTER_LINEAR;
        sampler_info.minFilter = VK_FILTER_LINEAR;
        sampler_info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;  // outside image bounds just use border color
        sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        sampler_info.minLod = -1000;
        sampler_info.maxLod = 1000;
        sampler_info.maxAnisotropy = 1.0f;
        err = vkCreateSampler(device, &sampler_info, nullptr, &m_sampler);
        Ui::checkVkResult(err);
    }

    // Create Descriptor Set using ImGUI's implementation
    m_descr_set = ImGui_ImplVulkan_AddTexture(m_sampler, m_image_view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

void Image::setData(const void* img_data) {
    auto device = Ui::App::getVkDevice();
    const uint32_t image_size = m_width * m_height * m_bytes_per_pixel;
    VkResult err;

    if (!m_upload_buffer) {
        // Create Upload Buffer
        {
            VkBufferCreateInfo buffer_info = {};
            buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
            buffer_info.size = image_size;
            buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
            buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            err = vkCreateBuffer(device, &buffer_info, nullptr, &m_upload_buffer);
            Ui::checkVkResult(err);
            VkMemoryRequirements req;
            vkGetBufferMemoryRequirements(device, m_upload_buffer, &req);
            VkMemoryAllocateInfo alloc_info = {};
            alloc_info.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            alloc_info.allocationSize = req.size;
            alloc_info.memoryTypeIndex = Ui::findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
            err = vkAllocateMemory(device, &alloc_info, nullptr, &m_upload_buffer_memory);
            Ui::checkVkResult(err);
            err = vkBindBufferMemory(device, m_upload_buffer, m_upload_buffer_memory, 0);
            Ui::checkVkResult(err);
        }
    }

    // Upload to Buffer
    {
        void* map = NULL;
        err = vkMapMemory(device, m_upload_buffer_memory, 0, image_size, 0, &map);
        Ui::checkVkResult(err);
        memcpy(map, img_data, image_size);
        VkMappedMemoryRange range[1] = {};
        range[0].sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
        range[0].memory = m_upload_buffer_memory;
        range[0].size = image_size;
        err = vkFlushMappedMemoryRanges(device, 1, range);
        Ui::checkVkResult(err);
        vkUnmapMemory(device, m_upload_buffer_memory);
    }

    // Copy to Image
    {
        VkCommandBuffer command_buffer = Ui::App::getVkCommandBuffer();

        VkImageMemoryBarrier copy_barrier[1] = {};
        copy_barrier[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        copy_barrier[0].dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        copy_barrier[0].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        copy_barrier[0].newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        copy_barrier[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        copy_barrier[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        copy_barrier[0].image = m_image;
        copy_barrier[0].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        copy_barrier[0].subresourceRange.levelCount = 1;
        copy_barrier[0].subresourceRange.layerCount = 1;
        vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_HOST_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, NULL, 0,
                             NULL, 1, copy_barrier);

        VkBufferImageCopy region = {};
        region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        region.imageSubresource.layerCount = 1;
        region.imageExtent.width = m_width;
        region.imageExtent.height = m_height;
        region.imageExtent.depth = 1;
        vkCmdCopyBufferToImage(command_buffer, m_upload_buffer, m_image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                               &region);

        VkImageMemoryBarrier use_barrier[1] = {};
        use_barrier[0].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        use_barrier[0].srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        use_barrier[0].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        use_barrier[0].oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        use_barrier[0].newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        use_barrier[0].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        use_barrier[0].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        use_barrier[0].image = m_image;
        use_barrier[0].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        use_barrier[0].subresourceRange.levelCount = 1;
        use_barrier[0].subresourceRange.layerCount = 1;
        vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0,
                             0, NULL, 0, NULL, 1, use_barrier);

        Ui::App::endVkCommandBuffer(command_buffer);
    }
}

void Image::resize(uint32_t width, uint32_t height) {
    if (m_width != width || m_height != height) {
        m_width = width;
        m_height = height;
        cleanup();
        initialize();
    }
}

void Image::cleanup() {
    const auto device = Ui::App::getVkDevice();

    Ui::App::addToCleanupQueue([device = device, sampler = m_sampler, image_view = m_image_view, image = m_image,
                                image_memory = m_image_memory, upload_buffer = m_upload_buffer,
                                upload_buffer_memory = m_upload_buffer_memory]() {
        vkDestroySampler(device, sampler, nullptr);
        vkDestroyImageView(device, image_view, nullptr);
        vkDestroyImage(device, image, nullptr);
        vkFreeMemory(device, image_memory, nullptr);
        vkDestroyBuffer(device, upload_buffer, nullptr);
        vkFreeMemory(device, upload_buffer_memory, nullptr);
    });
    // ImGui_ImplVulkan_RemoveTexture(m_descr_set);

    m_image_view = nullptr;
    m_image = nullptr;
    m_image_memory = nullptr;
    m_sampler = nullptr;
    m_upload_buffer = nullptr;
    m_upload_buffer_memory = nullptr;
}
