#pragma once

#include <stb_image.h>

#include <boost/numeric/odeint.hpp>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <iostream>
#include <map>
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
    glm::vec3 origin_cam_dir;  // helper var, cam direction to be used for some projections in hit() methods
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

    virtual bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) = 0;
    virtual void cleanUp() {}

    std::string name;
    glm::vec3 position;
    color color;
    MaterialType material = MaterialType::Lambertian;  // TODO: use pointer to material class?
};

struct Plane : Object {
    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) override {
        return false;
    }
};

struct Cube : Object {
    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) override {
        return false;
    }

    float width;
    float height;
};

struct Sphere : Object {
    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) override {
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

    float radius = 1.f;
};

/// Based on the wormhole metric described in https://arxiv.org/abs/1502.03809,
/// see section "Dneg wormhole without gravity". Numeric geodesics integration
/// is performed by calculating the hamiltonian of each ray.
struct ExtendedEllisWormhole : Object {
    using mat44 = glm::mat4x4;
    using vec4 = glm::vec4;
    using vec3 = glm::vec3;
    using vec2 = glm::vec2;

    // Procedure used for faster rendering:
    // The final angular position of each ray only depends on the angular momentum in the eq. plane,
    // thus we can calculate it once and reuse it, IF the coordinate system is rotated accordingly.
    // With the mapping angle_dir_yz -> (l, phi_eq) and the stored rotation per ray,
    // the final position (l, theta, phi) can be restored.
    using TargetCoordLookup = std::map<float, vec2>;

    enum CelestialSphereType { Saturn = 0, Galaxy1, Galaxy2, Eve1, Eve2, Eve3, Eve4, Eve5 };

    ExtendedEllisWormhole() { loadImages(); }

    bool hit(const Ray& ray, const LightSources& lights, float t_min, float t_max, RayHitRecord& rec) override {
        // unused for wormhole rendering
        (void)lights;
        (void)t_min;
        (void)t_max;

        // z-dir towards wormhole, y-dir right-handed
        auto cartToSpherical = [](const vec3& v) {
            auto r = sqrt(dot(v, v));
            // return vec3(r, atan2(v.x, sqrt(v.z * v.z + v.y * v.y)), atan2(v.y, v.z));
            auto y = (v.y == 0.f) ? std::numeric_limits<float>::epsilon() : v.y;
            return vec3(r, acos(v.x / r), atan2(v.y, v.z));
        };

        auto sphericalToCart = [](const vec3& v) {
            return vec3(v.x * cos(v.y), v.x * sin(v.y) * sin(v.z), v.x * sin(v.y) * cos(v.z));
        };

        // calculate ray direction unit vector N in camera's local sky
        // const auto ray_dir_spherical = cartToSpherical(ray.dir);
        // const float theta_cs = ray_dir_spherical.y;
        // const float phi_cs = ray_dir_spherical.z;
        // const auto N = vec3(cos(theta_cs), sin(theta_cs) * sin(phi_cs), sin(theta_cs) * cos(phi_cs));

        // get rotations for cam position and direction relative to z-axis:

        const auto cam_dir = ray.origin_cam_dir;
        const float pos_angle_to_neg_z_axis = atan2(ray.orig.y, -ray.orig.z);
        const float dir_angle_to_z_axis = atan2(cam_dir.y, cam_dir.z);

        // transformation to rotate original position to z-axis system
        const auto t1 = glm::translate(glm::eulerAngleX(-pos_angle_to_neg_z_axis), -ray.orig);
        const auto raypos_rtza_T_rayorigin = glm::translate(t1, ray.orig);

        // cam dir rotated to z-axis (rtza)
        const auto raydir_rtza_T_raydir = glm::eulerAngleX(dir_angle_to_z_axis);
        const auto raydir_rtza = vec3(raydir_rtza_T_raydir * vec4(ray.dir, 1.0));

        vec3 ray_march_pos = vec3(raypos_rtza_T_rayorigin * vec4(ray.orig, 1.f));
        ray_march_pos = cartToSpherical(ray_march_pos);
        ray_march_pos.x = l_cam_origin;

        if (compute_rays_in_eq_plane) {
            float angle_to_eq_plane = atan2(abs(raydir_rtza.x), abs(raydir_rtza.y));
            float rotation_dir = (raydir_rtza.x * raydir_rtza.y > 0.f) ? 1.f : -1.f;

            // rotation to equatorial plane
            const auto raydir_rtza_eq_plane_T_raydir_rtza = glm::eulerAngleZ(rotation_dir * angle_to_eq_plane);

            const auto normalized_ray_dir = vec3(raydir_rtza_eq_plane_T_raydir_rtza * vec4(raydir_rtza, 1.0));

            // take rounded angle between ray direction in eq. plane and z-axis as key for lookup table
            float angle_norm_ray_dir_to_z_axis = atan2(normalized_ray_dir.y, normalized_ray_dir.z);
            angle_norm_ray_dir_to_z_axis = std::ceil(angle_norm_ray_dir_to_z_axis * 1000.0) / 1000.0;

            // direction of propagation in global spherical base coordinates,
            // assuming transformed origin at pos = (0,0,-z), with z > 0.
            // auto ray_march_dir = vec3(-N.z, N.x, -N.y);
            auto ray_march_dir = vec3(-normalized_ray_dir.z, normalized_ray_dir.x, -normalized_ray_dir.y);

            const auto it = eq_target_coord_lookup.find(angle_norm_ray_dir_to_z_axis);
            if (it != eq_target_coord_lookup.end()) {
                const vec2 eq_target_coords = it->second;
                const float eq_l = eq_target_coords.x;
                const float eq_phi = eq_target_coords.y;
                // get ray_march_pos from eq. plane target point (eq_l, eq_phi)
                ray_march_pos = vec3(eq_l, kPi / 2.f, eq_phi);
            } else {
                float time = 0.f;

                // TODO: Make this an option
                // This flips the camera direction to point to the wormhole after the transition to the upper sphere
                if (l_cam_origin > a) {
                    ray_march_dir.x *= -1.f;
                }

                traceGeodesic(ray_march_pos, ray_march_dir, time);

                eq_target_coord_lookup.insert(
                    std::make_pair(angle_norm_ray_dir_to_z_axis, vec2(ray_march_pos.x, ray_march_pos.z)));
            }

            // sign needs be preserved, as l will become r > 0 after coord. transformation
            const auto l = ray_march_pos.x;
            ray_march_pos.x = radiusFromTraveledDistance(l);
            ray_march_pos = sphericalToCart(ray_march_pos);

            // rotate back from eq. plane
            ray_march_pos = vec3(inverse(raydir_rtza_eq_plane_T_raydir_rtza) * vec4(ray_march_pos, 1.f));

            // rotate back around x-axis to original location
            ray_march_pos = vec3(inverse(raypos_rtza_T_rayorigin) * vec4(ray_march_pos, 1.f));

            ray_march_pos = cartToSpherical(ray_march_pos);
            ray_march_pos.x = l;
        } else {
            // Assuming the camera to be directed towards the wormhole "center" along the z-axis.
            // The cartesian ray direction components can be reused to define the initial velocities
            // in the global spherical coordinate system:
            auto ray_march_dir = vec3(-raydir_rtza.z, raydir_rtza.x, -raydir_rtza.y);

            float time = 0.f;
            traceGeodesic(ray_march_pos, ray_march_dir, time);

            // sign needs be preserved, as l will become r > 0 after coord. transformation
            const auto l = ray_march_pos.x;
            ray_march_pos.x = radiusFromTraveledDistance(l);
            ray_march_pos = sphericalToCart(ray_march_pos);

            // rotate back around x-axis to original location
            ray_march_pos = vec3(inverse(raypos_rtza_T_rayorigin) * vec4(ray_march_pos, 1.f));
            ray_march_pos = cartToSpherical(ray_march_pos);
            ray_march_pos.x = l;
        }
        // TRACEIT_LOG_INFO("final sph pos orignal = (" << ray_march_pos.x << ", " << ray_march_pos.y << ", "
        //                                              << ray_march_pos.z << ")");
        rec.color = spherePixelColor(ray_march_pos);

        return true;
    }

    void cleanUp() override { eq_target_coord_lookup.clear(); }

    // pos, dir given in spherical coordinates
    void traceGeodesic(vec3& pos, const vec3& dir, float& time) const {
        auto x = vec4(time, pos);  // x = (t, l, theta, phi) = (x, y, z, w)
        auto p = canonicalMomenta(x, dir);

        // ray's constants of motion
        const float b = p.w;
        const float r = radiusFromTraveledDistance(x.y);
        const float B_sq = r * r * (dir.y * dir.y + dir.z * dir.z);

        using StateType = std::array<float, 5>;

        // state_x = (l, theta, phi, p_l, p_theta) = (x.y, x.z, x.w, p.y, p.z)
        auto rayEquations = [b, B_sq, this](const StateType& state_x, StateType& dxdt, const double /* t */) {
            // r + drdl
            float r = radiusFromTraveledDistance(state_x[0]);
            float h = 0.001;
            float drdl = (radiusFromTraveledDistance(state_x[0] + h) - r) / h;

            const float ir = 1 / r;
            const float ir_sq = ir * ir;
            const float theta = state_x[1];
            const float sin_theta = sin(theta);

            dxdt[0] = state_x[3];                                      // dl/dt
            dxdt[1] = state_x[4] * ir_sq;                              // dtheta/dt
            dxdt[2] = b * ir_sq / pow(sin_theta, 2);                   // dphi/dt
            dxdt[3] = B_sq * ir_sq * ir * drdl;                        // dp_l/dt
            dxdt[4] = b * b * ir_sq * cos(theta) / pow(sin_theta, 3);  // dp_theta/dt
        };

        boost::numeric::odeint::runge_kutta4<StateType> stepper;
        StateType state_x{x.y, x.z, x.w, p.y, p.z};
        const double dt = -0.1;
        for (double ts = 0.0; ts > -20.0; ts += dt) {
            stepper.do_step(rayEquations, state_x, ts, dt);
        }

        pos = vec3(state_x[0], state_x[1], state_x[2]);
    }

    vec4 canonicalMomenta(const vec4& x, const vec3& dir) const {
        const float r = radiusFromTraveledDistance(x.y);
        const float theta = x.z;
        return vec4(-1.f, dir.x, r * dir.y, r * sin(theta) * dir.z);
    }

    float radiusFromTraveledDistance(float l) const {
        float _x_top = 2 * abs(l) - a;
        float _x_bot = kPi * M;
        float _x = _x_top / _x_bot;

        float r = 0.f;
        if (abs(l) > a) {
            r = rho + M * (_x * atan2(_x_top, _x_bot) - 0.5f * log(1 + _x * _x));
        } else {
            r = rho;
        }
        return r;
    }

    void initTraveledDistance(float l) { l_cam_origin = l; }

    void updateTraveledDistance(float dr) {
        const float r = radiusFromTraveledDistance(l_cam_origin);
        l_cam_origin += dr * 1 / sqrt(1 - 2 * M / r);
        TRACEIT_LOG_INFO("l = " << l_cam_origin);
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
                case CelestialSphereType::Eve1:
                    return WORMHOLE_4_IMG_SPHERE_PATH;
                case CelestialSphereType::Eve2:
                    return WORMHOLE_5_IMG_SPHERE_PATH;
                case CelestialSphereType::Eve3:
                    return WORMHOLE_6_IMG_SPHERE_PATH;
                case CelestialSphereType::Eve4:
                    return WORMHOLE_7_IMG_SPHERE_PATH;
                case CelestialSphereType::Eve5:
                    return WORMHOLE_8_IMG_SPHERE_PATH;
                default:
                    return WORMHOLE_1_IMG_SPHERE_PATH;
            }
        };

        // Load from disk into a raw RGB buffer
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

        // Normalize angles to prevent out-of-bounds x,y pixel calc. below
        auto normalize0TokPi = [](float angle, const float factor = 1.f) {
            angle = fmod(angle, factor * kPi);
            if (angle < 0.f) angle += factor * kPi;
            return angle;
        };

        if (theta < 0.f) return pixel_color;  // just in case
        theta = normalize0TokPi(theta);
        phi = normalize0TokPi(phi, 2.f);
        if (theta < 0.f || theta > kPi || phi < 0.f || phi > 2 * kPi || isnan(theta) || isnan(phi)) {
            TRACEIT_LOG_ERROR("Angles exceed limits:"
                              << "theta=" << theta << ", phi=" << phi);
            exit(1);
        }

        auto rgbaToColor = [](unsigned char* pixel_offset) {
            vec4 color(0.f);
            color.r = static_cast<float>(pixel_offset[0]) / 255;
            color.g = static_cast<float>(pixel_offset[1]) / 255;
            color.b = static_cast<float>(pixel_offset[2]) / 255;
            color.a = 1.f;
            return color;
        };

        // phi range = [0,2*pi], theta range = [0, pi]
        if (l < 0.f && image_data_lower_sphere) {
            auto x = static_cast<int>(image_lower_sphere_width * phi / (2 * kPi));
            auto y = static_cast<int>(image_lower_sphere_height * theta / kPi);
            unsigned char* pixel_offset =
                image_data_lower_sphere + (x + y * image_lower_sphere_width) * bytes_per_pixel;
            pixel_color = rgbaToColor(pixel_offset);
        } else if (image_data_upper_sphere) {
            auto x = static_cast<int>(image_upper_sphere_width * phi / (2 * kPi));
            auto y = static_cast<int>(image_upper_sphere_height * theta / kPi);
            unsigned char* pixel_offset =
                image_data_upper_sphere + (x + y * image_upper_sphere_width) * bytes_per_pixel;
            pixel_color = rgbaToColor(pixel_offset);
        }

        return pixel_color;
    }

    float a = 0.1f;   // half wormhole length
    float rho = 2.f;  // wormhole throat radius
    float M = 0.2f;   // mass, representing the gentleness of the interior-to-exterior transition of the wormhole

    float l_cam_origin;  // to determine in which celestial sphere we are for ellis wormhole metric

    // Only possible if camera points towards wormhole center!
    // Rotates ray before start of integration to eq. plane and reverts that before pixel color retrieval
    bool compute_rays_in_eq_plane = true;
    TargetCoordLookup eq_target_coord_lookup;

    const uint32_t bytes_per_pixel = 3;
    CelestialSphereType lower_celestial_sphere{CelestialSphereType::Saturn};
    CelestialSphereType upper_celestial_sphere{CelestialSphereType::Galaxy2};
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

    void cleanUp() {
        for (const auto& object_variant : m_objects) {
            std::visit([&](auto&& obj) { obj.cleanUp(); }, *object_variant);
        }
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
