#include "camera.h"

using namespace glm;

Camera::Camera(vec3 origin, vec3 direction, float near_clip, float far_clip)
    : m_origin(origin),
      m_direction(normalize(direction)),
      m_near_clip(near_clip),
      m_far_clip(far_clip),
      m_proj_mat(),
      m_proj_mat_inv(),
      m_view_mat(),
      m_view_mat_inv() {
    m_ray_directions.resize(m_viewport_width * m_viewport_height);
}

void Camera::updatePose() {
    // TODO: add movement of camera
    if (!m_pose_initialized) {
        updateViewMatrix();
        m_pose_initialized = true;
    }
}

void Camera::refresh(uint32_t viewport_width, uint32_t viewport_height) {
    if (m_viewport_width != viewport_width || m_viewport_height != viewport_height) {
        m_viewport_width = viewport_width;
        m_viewport_height = viewport_height;
        m_aspect_ratio = static_cast<float>(viewport_width) / viewport_height;
        updateProjectionMatrix();
        updateRayDirections();
    }
}

void Camera::updateProjectionMatrix() {
    m_proj_mat = perspectiveFov(kFovy, static_cast<float>(m_viewport_width), static_cast<float>(m_viewport_height),
                                m_near_clip, m_far_clip);
    m_proj_mat_inv = inverse(m_proj_mat);
}

void Camera::updateViewMatrix() {
    m_view_mat = lookAt(m_origin, m_origin + m_direction, vec3(0.f, 1.f, 0.f));  // (0,1,0) = up-vector in world space
    m_view_mat_inv = inverse(m_view_mat);
}

vec2 Camera::clipSpaceCoordinates(uint32_t x, uint32_t y) const {
    vec2 clip_coord(static_cast<float>(x) / static_cast<float>(m_viewport_width),
                    static_cast<float>(y) / static_cast<float>(m_viewport_height));
    clip_coord = clip_coord * 2.f - 1.f;
    return clip_coord;
}

void Camera::updateRayDirections() {
    m_ray_directions.resize(m_viewport_width * m_viewport_height);

    /// x_clip = M_proj * M_view * M_model * M_local * x_local => x_clip = M_proj * M_view * x_world
    /// (see e.g. https://learnopengl.com/Getting-started/Camera)
    /// Transforming from clip to world space: x_world = M_view^-1 * M_proj^-1 * x_clip
    for (uint32_t y = 0; y < m_viewport_height; y++) {
        for (uint32_t x = 0; x < m_viewport_width; x++) {
            const auto clip_coords = clipSpaceCoordinates(x, y);
            const vec4 world_coords = m_proj_mat_inv * vec4(clip_coords.x, clip_coords.y, 1.f, 1.f);
            const auto normalized_cart_world_coords =
                normalize(vec3(world_coords) /= world_coords.w);  // perspective divide
            const auto ray_direction = vec3(m_view_mat_inv * vec4(normalized_cart_world_coords, 0));
            m_ray_directions[x + y * m_viewport_width] = ray_direction;
        }
    }
}
