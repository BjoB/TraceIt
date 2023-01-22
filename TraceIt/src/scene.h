#pragma once

#include <glm/glm.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

#include "log.h"

using color = glm::vec4;

constexpr float kPi = static_cast<float>(3.14159265358979323846);

// Rays

class Ray {
   public:
    using Scalar = glm::vec3::value_type;

    Ray(const glm::vec3& origin, const glm::vec3& direction) : orig(origin), dir(direction) {}

    glm::vec3 at(Scalar t) const { return orig + t * dir; }

   public:
    glm::vec3 orig;
    glm::vec3 dir;
};

struct RayHitRecord {
    void setFaceNormal(const Ray& ray, const glm::vec3& outward_normal) {
        front_face = dot(ray.dir, outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }

    glm::vec3 position;
    glm::vec3 normal;
    color color;
    float t;
    bool front_face;
};

// Lights

struct LightSource {
    virtual ~LightSource() = default;

    std::string name;
    glm::vec3 position;
    glm::vec3 color = glm::vec3(0.8f, 0.5f, 0.2f);  // orange
    float intensity = 0.5f;
};

struct PointLight : LightSource {};

using LightSources = std::vector<std::shared_ptr<LightSource>>;

// Hittable objects

enum MaterialType { Lambertian, Metal };

struct Object {
    virtual ~Object() = default;

    virtual bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) const = 0;

    std::string name;
    glm::vec3 position;
    color color;
    MaterialType material = MaterialType::Lambertian;  // TODO: use pointer to material class?
};

struct Plane : Object {
    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) const override {
        return false;
    }
};

struct Cube : Object {
    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) const override {
        return false;
    }

    float width;
    float height;
};

struct Sphere : Object {
    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) const override {
        const glm::vec3 sphere_center_to_ray_orig = ray.orig - position;
        const auto a = dot(ray.dir, ray.dir);
        const auto half_b = dot(sphere_center_to_ray_orig, ray.dir);
        const auto c = dot(sphere_center_to_ray_orig, sphere_center_to_ray_orig) - radius * radius;
        const auto discriminant = half_b * half_b - a * c;
        if (discriminant < 0) {
            rec.t = -1.f;
            return false;
        }
        const auto sqrt_discr = sqrt(discriminant);

        auto inAllowedRange = [&](float t) { return (t < t_min || t > t_max) ? false : true; };

        auto t = (-half_b - sqrt_discr) / a;
        if (!inAllowedRange(t)) {
            t = (-half_b + sqrt_discr) / a;
            if (!inAllowedRange(t)) {
                return false;
            }
        }

        // set general hitpoint properties
        rec.t = t;
        rec.position = ray.at(t);
        const auto outward_normal = (rec.position - position) / this->radius;
        rec.setFaceNormal(ray, outward_normal);

        const auto obj_color = glm::vec3(this->color);
        // Calculate diffuse surface color:
        // Li = light_intensity * light_color / 4*pi*r^2
        // diffuse_color = albedo/pi * Li * cos(theta)
        for (const auto& light : lights) {
            const auto hitpoint_to_light = light->position - rec.position;
            const auto hitpoint_to_light_dist_sq = glm::dot(hitpoint_to_light, hitpoint_to_light);
            const auto dir_towards_light = glm::normalize(hitpoint_to_light);
            const auto diffuse_color =
                obj_color * light->intensity * light->color * std::max(0.f, glm::dot(rec.normal, dir_towards_light));
            // TODO: put (4 * kPi * hitpoint_to_light_dist_sq)  somewhere?
            rec.color = glm::vec4(diffuse_color, 1.f);
        }

        return true;
    }

    float radius;
};

// Scene handling

using HittableObjectVariant = std::variant<Plane, Cube, Sphere>;

class Scene {
   public:
    Scene() {}

    bool addObject(const std::string& object_type) {
        const auto next_light_id = static_cast<int>(m_lights.size()) + 1;
        const auto next_light_id_str = std::to_string(next_light_id);
        const auto next_obj_id = static_cast<int>(m_objects.size()) + 1;
        const auto next_obj_id_str = std::to_string(next_obj_id);

        if (object_type == "Pointlight")
            addLightSource<PointLight>(std::string("Pointlight") + next_light_id_str);
        else if (object_type == "Plane")
            addHittableObject<Plane>(std::string("Plane") + next_obj_id_str);
        else if (object_type == "Cube")
            addHittableObject<Cube>(std::string("Cube") + next_obj_id_str);
        else if (object_type == "Sphere")
            addHittableObject<Sphere>("Sphere" + next_obj_id_str);
        else {
            TRACEIT_LOG_ERROR("Unknown object type " << object_type << ".");
            return false;
        }
        return true;
    }

    auto& lights() const { return m_lights; }

    auto& objects() const { return m_objects; }

    bool hit(const Ray& ray, double t_min, double t_max, RayHitRecord& rec) const {
        RayHitRecord temp_rec;
        bool hit_any_object = false;
        auto closest_t = t_max;

        for (const auto& object_variant : m_objects) {
            if (std::visit([&](auto&& obj) { return obj.hit(ray, m_lights, t_min, closest_t, temp_rec); },
                           *object_variant)) {
                hit_any_object = true;
                closest_t = temp_rec.t;
                rec = temp_rec;
            }
        }
        return hit_any_object;
    }

    std::vector<std::string> availableObjectTypes() const { return m_available_object_types; }

   private:
    template <class LightSourceT>
    void addLightSource(const std::string& light_name = "New light") {
        static_assert(std::is_base_of_v<LightSource, LightSourceT>,
                      "Light source need to derive from LightSource struct!");
        m_lights.push_back(std::make_shared<LightSourceT>());
        auto& added_light = *m_lights.back();
        added_light.name = light_name;
    }

    template <class ObjectT>
    void addHittableObject(const std::string& obj_name = "New object") {
        static_assert(std::is_base_of_v<Object, ObjectT>, "Scene objects need to derive from Object struct!");
        m_objects.push_back(std::make_shared<HittableObjectVariant>(ObjectT{}));
        auto& added_obj_variant = *m_objects.back();
        std::visit([&](auto&& obj_variant) { obj_variant.name = obj_name; }, added_obj_variant);
    }

   private:
    const std::vector<std::string> m_available_object_types{"Pointlight", "Plane", "Cube",
                                                            "Sphere"};  // names of addable object types
    std::vector<std::shared_ptr<LightSource>> m_lights;
    std::vector<std::shared_ptr<HittableObjectVariant>> m_objects;
};
