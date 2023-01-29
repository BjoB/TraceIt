#pragma once

#include <stb_image.h>

#include <cmath>
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

/// Based on the wormhole metric described in https://arxiv.org/abs/1502.03809,
/// see section "Dneg wormhole without gravity". Numeric geodesics integration
/// is performed using the hamiltonian formulation of the metric.
struct ExtendedEllisWormhole : Object {
    using mat44 = glm::mat4x4;
    using vec4 = glm::vec4;
    using vec3 = glm::vec3;

    enum class CelestialSphereType { Saturn, Galaxy1, Galaxy2 };

    ExtendedEllisWormhole() { loadImages(); }

    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) const override {
        // unused for wormhole rendering
        (void)lights;
        (void)t_min;
        (void)t_max;

        // z-dir: away from camera, y-dir: right-handed
        auto cartToSpherical = [](const vec3& v) {
            auto r = sqrt(dot(v, v));
            // return vec3(r, atan2(v.x, sqrt(v.z * v.z + v.y * v.y)), atan2(v.y, v.z));
            return vec3(r, acos(v.x / r), atan2(v.y, v.z));
        };

        vec3 ray_march_pos = cartToSpherical(ray.orig);
        vec3 ray_march_dir = cartToSpherical(ray.dir);
        float time = 0.f;
        traceGeodesic(ray_march_pos, ray_march_dir, time);
        rec.color = spherePixelColor(ray_march_pos);
        // rec.color = spherePixelColor(ray_march_dir);

        return true;
    }

    // pos, dir given in spherical coordinates
    void traceGeodesic(vec3& pos, vec3& dir, float& time) const {
        auto x = vec4(time, pos);
        auto p = initialMomenta(x, dir);

        const int steps = 256;
        for (int i = 0; i < steps; ++i) {
            integrateOneStep(x, p);
        }

        pos = vec3(x.y, x.z, x.w);
        auto dxdt = inverse(metric(x)) * p;
        dir = normalize(vec3(dxdt.y, dxdt.z, dxdt.w));
        time = x.x;
    }

    vec4 initialMomenta(vec4 x, vec3 dir) const { return metric(x) * vec4(1.0, normalize(dir)); }

    void integrateOneStep(vec4& x, vec4& p) const {
        // using euler integration
        const float h = 0.1f;
        p = p - h * hamiltonianGrad(x, p);   // dp_i/dt = -grad_i(H(x))
        x = x + h * inverse(metric(x)) * p;  // dx^i/dt = g^{ij}(x) * p_j
    }

    vec4 hamiltonianGrad(vec4 x, vec4 p) const {
        const float eps = 0.001f;
        return (vec4(hamiltonian(x + vec4(eps, 0, 0, 0), p), hamiltonian(x + vec4(0, eps, 0, 0), p),
                     hamiltonian(x + vec4(0, 0, eps, 0), p), hamiltonian(x + vec4(0, 0, 0, eps), p)) -
                hamiltonian(x, p)) /
               eps;
    }

    float hamiltonian(vec4 x, vec4 p) const {
        mat44 g_inv = inverse(metric(x));
        return 0.5f * glm::dot(g_inv * p, p);
    }

    mat44 metric(vec4 x) const {
        // x = (t, l, theta, phi) = (x, y, z, w)
        float _x_top = 2 * abs(x.y) - a;
        float _x_bot = kPi * M;
        float _x = _x_top / _x_bot;

        float r = 0.f;
        if (abs(x.y) > a) {
            r = rho + M * (_x * atan2(_x_top, _x_bot) - 0.5f * log(1 + _x * _x));
        } else {
            r = rho;
        }

        // coordinate singularities theta=0 and theta=pi, leading to non-invertible metric!
        return mat44(-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, r * r, 0, 0, 0, 0, powf(r * sin(x.z), 2));
    }

    void loadImages() {
        if (image_data_lower_sphere) {
            stbi_image_free(image_data_lower_sphere);
            image_data_lower_sphere = nullptr;
        }
        if (image_data_upper_sphere) {
            stbi_image_free(image_data_upper_sphere);
            image_data_upper_sphere = nullptr;
        }

        auto getImgPath = [](const CelestialSphereType type) -> const char* {
            switch (type) {
                case CelestialSphereType::Saturn:
                    return WORMHOLE_1_IMG_SPHERE_PATH;
                case CelestialSphereType::Galaxy1:
                    return WORMHOLE_2_IMG_SPHERE_PATH;
                case CelestialSphereType::Galaxy2:
                    return WORMHOLE_3_IMG_SPHERE_PATH;
                default:
                    return WORMHOLE_1_IMG_SPHERE_PATH;
            }
        };

        // Load from disk into a raw RGBA buffer
        image_data_lower_sphere = stbi_load(getImgPath(lower_celestial_sphere), &image_lower_sphere_width,
                                            &image_lower_sphere_height, nullptr, bytes_per_pixel);
        image_data_upper_sphere = stbi_load(getImgPath(upper_celestial_sphere), &image_upper_sphere_width,
                                            &image_upper_sphere_height, nullptr, bytes_per_pixel);
    }

    vec4 spherePixelColor(vec3 spherical_pos) const {
        vec4 pixel_color(0.f);
        auto l = spherical_pos.x;
        auto theta = spherical_pos.y;
        auto phi = spherical_pos.z;

        auto rgbaToColor = [](unsigned char* pixel_offset) {
            vec4 color(0.f);
            color.r = static_cast<float>(pixel_offset[0]) / 255;
            color.g = static_cast<float>(pixel_offset[1]) / 255;
            color.b = static_cast<float>(pixel_offset[2]) / 255;
            color.a = 1.f;
            return color;
        };

        // phi range = [-pi,pi], theta range = [0, pi]
        if (l > 0.f && image_data_lower_sphere) {
            auto x = static_cast<int>(image_lower_sphere_width * (phi - kPi) / (2 * kPi));
            auto y = static_cast<int>(image_lower_sphere_height * (theta) / kPi);
            unsigned char* pixel_offset =
                image_data_lower_sphere + (x + y * image_lower_sphere_width) * bytes_per_pixel;
            pixel_color = rgbaToColor(pixel_offset);
        } else if (image_data_upper_sphere) {
            auto x = static_cast<int>(image_upper_sphere_width * (phi - kPi) / (2 * kPi));
            auto y = static_cast<int>(image_upper_sphere_height * (theta) / kPi);
            unsigned char* pixel_offset =
                image_data_upper_sphere + (x + y * image_upper_sphere_width) * bytes_per_pixel;
            pixel_color = rgbaToColor(pixel_offset);
        }

        return pixel_color;
    }

    float a = 1.f;    // half wormhole length
    float rho = 1.f;  // wormhole throat radius
    float M = 1.0f;   // mass, representing the gentleness of the interior-to-exterior transition of the wormhole

    const uint32_t bytes_per_pixel = 3;
    CelestialSphereType lower_celestial_sphere{CelestialSphereType::Galaxy2};
    CelestialSphereType upper_celestial_sphere{CelestialSphereType::Saturn};
    unsigned char* image_data_lower_sphere = nullptr;
    unsigned char* image_data_upper_sphere = nullptr;
    int image_lower_sphere_width = 0;
    int image_lower_sphere_height = 0;
    int image_upper_sphere_width = 0;
    int image_upper_sphere_height = 0;
};

// Scene handling

using HittableObjectVariant = std::variant<Plane, Cube, Sphere, ExtendedEllisWormhole>;

class Scene {
   public:
    Scene() {}

    bool addObject(const std::string& object_type) {
        if (object_type == "Pointlight") {
            addLightSource<PointLight>(std::string("Pointlight") + nextId(m_lights));

        } else {
            const auto next_obj_id = nextId(m_objects);
            if (object_type == "Plane")
                addHittableObject<Plane>(std::string("Plane") + next_obj_id);
            else if (object_type == "Cube")
                addHittableObject<Cube>(std::string("Cube") + next_obj_id);
            else if (object_type == "Sphere")
                addHittableObject<Sphere>("Sphere" + next_obj_id);
            else if (object_type == "Wormhole")
                addHittableObject<ExtendedEllisWormhole>(std::string("Wormhole") + next_obj_id);
            else {
                TRACEIT_LOG_ERROR("Unknown object type " << object_type << ".");
                return false;
            }
        }
        return true;
    }

    template <typename ObjectT>
    void eraseLightSource(std::shared_ptr<ObjectT>& obj_to_erase) {
        m_lights.erase(std::remove(m_lights.begin(), m_lights.end(), obj_to_erase));
    }

    template <typename DerivedObjectT>
    void eraseHittableObject(std::shared_ptr<DerivedObjectT>& obj_to_erase) {
        m_objects.erase(std::remove(m_objects.begin(), m_objects.end(), obj_to_erase));
    }

    auto& lights() { return m_lights; }

    auto& objects() { return m_objects; }

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

    template <typename T>
    std::string nextId(const std::vector<std::shared_ptr<T>>& scene_objects) {
        std::string last_scene_obj_name = "0";
        size_t num_scene_objects = 0;
        if constexpr (std::is_base_of_v<LightSource, T>) {
            num_scene_objects = m_lights.size();
            if (num_scene_objects > 0) last_scene_obj_name = m_lights.back()->name;
        } else if constexpr (std::is_base_of_v<HittableObjectVariant, T>) {
            num_scene_objects = m_objects.size();
            if (num_scene_objects > 0) {
                std::visit([&](const auto& obj_variant) { last_scene_obj_name = obj_variant.name; }, *m_objects.back());
            }
        } else {
            static_assert("Unsupported scene object type!");
        }
        const int last_id = stoi(last_scene_obj_name.substr(last_scene_obj_name.find_first_of("0123456789")));
        return std::to_string(std::max(static_cast<int>(num_scene_objects) + 1, last_id + 1));
    }

   private:
    const std::vector<std::string> m_available_object_types{"Pointlight", "Plane", "Cube", "Sphere",
                                                            "Wormhole"};  // names of addable object types
    std::vector<std::shared_ptr<LightSource>> m_lights;
    std::vector<std::shared_ptr<HittableObjectVariant>> m_objects;
};
