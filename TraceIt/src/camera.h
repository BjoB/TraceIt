#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>

using namespace glm;

class Camera {
   public:
    Camera(vec3 origin, vec3 direction, float near_clip = 0.1f, float far_clip = 100.f);
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;

    void updatePose();

    void refresh(uint32_t viewport_width, uint32_t viewport_height);

    vec3 position() const { return m_origin; }

    vec3 direction() const { return m_direction; }

    vec3 rayDirection(uint32_t x, uint32_t y) const { return m_ray_directions[x + y * m_viewport_width]; }

   private:
    void updateProjectionMatrix();

    void updateViewMatrix();

    vec2 clipSpaceCoordinates(uint32_t x, uint32_t y) const;

    void updateRayDirections();

   private:
    const float kFovy = radians(45.f);

    std::vector<vec3> m_ray_directions;
    uint32_t m_viewport_width = 0, m_viewport_height = 0;
    vec3 m_origin, m_direction;
    mat4 m_proj_mat, m_proj_mat_inv, m_view_mat, m_view_mat_inv;
    float m_near_clip = 0.f, m_far_clip = 0.f, m_aspect_ratio = 0.f;

    bool m_pose_initialized = false;
};
