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

using namespace glm;
using color = vec4;

class Ray {
   public:
    using Scalar = vec3::value_type;

    Ray(const vec3& origin, const vec3& direction) : orig(origin), dir(direction) {}

    vec3 at(Scalar t) const { return orig + t * dir; }

   public:
    vec3 orig;
    vec3 dir;
};

struct RayHitRecord {
    vec3 position;
    vec3 normal;
    color color;
    float t;
};

enum ObjectType {
    kPlane,
    kCube,
    kSphere,
    kLastObj,
};

enum MaterialType { Lambertian, Metal };

struct Object {
    virtual ~Object() = default;

    virtual bool hit(const Ray& ray, float t_min, float t_max, RayHitRecord& rec) const = 0;

    std::string name;
    vec3 position;
    color color;
    MaterialType material;
};

struct Plane : Object {
    bool hit(const Ray& ray, float t_min, float t_max, RayHitRecord& rec) const override { return false; }
};

struct Cube : Object {
    bool hit(const Ray& ray, float t_min, float t_max, RayHitRecord& rec) const override { return false; }

    float width;
    float height;
};

struct Sphere : Object {
    bool hit(const Ray& ray, float t_min, float t_max, RayHitRecord& rec) const override {
        const vec3 sphere_center_to_ray_orig = ray.orig - position;
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

        rec.t = t;
        rec.position = ray.at(t);
        rec.normal = (rec.position - position) / radius;  // as unit vector
        rec.color = color;
        return true;
    }

    float radius;
};

using SceneObject = std::variant<Plane, Cube, Sphere>;

class Scene {
   public:
    Scene() {}

    template <class ObjectT>
    void addObject(const std::string& obj_name = "New Object") {
        static_assert(std::is_base_of_v<Object, ObjectT>, "Scene objects need to derive from Object struct!");
        m_objects.push_back(std::make_shared<SceneObject>(ObjectT{}));
        auto& added_obj_variant = *m_objects.back();
        std::visit([&](auto&& obj_variant) { obj_variant.name = obj_name; }, added_obj_variant);
    }

    bool addObject(const std::string& object_type) {
        const auto next_id = static_cast<int>(m_objects.size()) + 1;
        const auto next_id_str = std::to_string(next_id);

        if (object_type == "Plane")
            addObject<Plane>(std::string("Plane") + next_id_str);
        else if (object_type == "Cube")
            addObject<Cube>(std::string("Cube") + next_id_str);
        else if (object_type == "Sphere")
            addObject<Sphere>("Sphere" + next_id_str);
        else {
            TRACEIT_LOG_ERROR("Unknown object type " << object_type << ".");
            return false;
        }
        return true;
    }

    auto& objects() const { return m_objects; }

    bool hit(const Ray& ray, double t_min, double t_max, RayHitRecord& rec) const {
        RayHitRecord temp_rec;
        bool hit_any_object = false;
        auto closest_t = t_max;

        for (auto object_variant : m_objects) {
            if (std::visit([&](auto&& obj) { return obj.hit(ray, t_min, closest_t, temp_rec); }, *object_variant)) {
                hit_any_object = true;
                closest_t = temp_rec.t;
                rec = temp_rec;
            }
        }
        return hit_any_object;
    }

    auto availableObjectTypes() const {
        std::vector<std::string> obj_names;
        for (auto& [_, val] : m_available_objects_map) {
            obj_names.push_back(std::string(val));
        }
        return obj_names;
    }

   private:
    const std::unordered_map<ObjectType, const char*> m_available_objects_map{
        {ObjectType::kPlane, "Plane"}, {ObjectType::kCube, "Cube"}, {ObjectType::kSphere, "Sphere"}};
    std::vector<std::shared_ptr<SceneObject>> m_objects;
};
