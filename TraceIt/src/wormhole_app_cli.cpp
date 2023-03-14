// TraceIt - CLI - Main Application

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <chrono>
#include <cxxopts.hpp>
#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "camera.h"
#include "log.h"
#include "renderer.h"
#include "scene.h"

constexpr float kOutputFrameRate = 30.f;

// z-dir towards wormhole, y-dir right-handed
auto cartToSpherical(const glm::vec3& v) {
    auto r = sqrt(dot(v, v));
    return glm::vec3(r, acos(v.x / r), atan2(v.y, v.z));
};

auto sphericalToCart(const glm::vec3& v) {
    return glm::vec3(v.x * cos(v.y), v.x * sin(v.y) * sin(v.z), v.x * sin(v.y) * cos(v.z));
};

class SceneSetup {
   public:
    SceneSetup(uint32_t image_width, uint32_t image_height, glm::vec3 cam_pos, glm::vec3 cam_dir,
               float wormhole_length_a, float wormhole_throat_radius_rho, float wormhole_mass_param,
               int lower_sphere_idx, int upper_sphere_idx)
        : m_render_image_width(image_width),
          m_render_image_height(image_height),
          m_camera(cam_pos, cam_dir),
          m_renderer(m_camera) {
        m_scene.addObject("Wormhole");
        m_wormhole_obj = &std::get<ExtendedEllisWormhole>(*m_scene.objects().back());
        m_wormhole_obj->a = wormhole_length_a;
        m_wormhole_obj->rho = wormhole_throat_radius_rho;
        m_wormhole_obj->M = wormhole_mass_param;
        setWormholeCelestialSpheres(lower_sphere_idx, upper_sphere_idx);
    }

    void update(glm::vec3 cam_pos, glm::vec3 cam_dir) {
        m_camera.updatePose(cam_pos, cam_dir);
        m_camera.refresh();
    }

    void render() {
        const auto start_time = std::chrono::high_resolution_clock::now();
        m_camera.refresh(m_render_image_width, m_render_image_height);
        m_renderer.refresh(m_render_image_width, m_render_image_height);
        m_renderer.render(m_scene);
        m_frame_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start_time);

        const auto img_filename = std::string("wh_") + std::to_string(m_cur_img_index) + std::string(".png");
        stbi_write_png(img_filename.c_str(), m_render_image_width, m_render_image_height, 4, m_renderer.imageData(),
                       m_render_image_width * 4);
        ++m_cur_img_index;
    }

    float curremtFrameTimeSecs() const { return static_cast<float>(m_frame_time_ms.count()) / 1e3; }

   private:
    void setWormholeCelestialSpheres(int lower_sphere_texture_idx, int upper_sphere_texture_idx) {
        auto setCelestialSphereEnum = [&](int sphere_texture_idx) {
            for (int i = 0; i < m_sphere_texture_names.size(); ++i) {
                if (strcmp(m_sphere_texture_names[i], m_sphere_texture_names[sphere_texture_idx])) {
                    m_wormhole_obj->lower_celestial_sphere = static_cast<ExtendedEllisWormhole::CelestialSphereType>(i);
                    break;
                }
            }
        };
        setCelestialSphereEnum(lower_sphere_texture_idx);
        setCelestialSphereEnum(upper_sphere_texture_idx);
        m_wormhole_obj->loadImages();
    }

    const std::vector<const char*> m_sphere_texture_names = {"Interstellar's Saturn Site",
                                                             "Galaxy String",
                                                             "Interstellar's Far Galaxy",
                                                             "Bluegrey Nebula",
                                                             "Brown Nebula",
                                                             "Green Nebula",
                                                             "Greenish-Brown Nebula",
                                                             "Smokey Nebula"};

    ExtendedEllisWormhole* m_wormhole_obj = nullptr;
    std::chrono::milliseconds m_frame_time_ms{0};
    std::string m_cur_lower_sphere_selection;
    std::string m_cur_upper_sphere_selection;
    Scene m_scene;
    Renderer m_renderer;
    Camera m_camera;
    uint32_t m_render_image_width = 0;
    uint32_t m_render_image_height = 0;
    uint32_t m_cur_img_index = 0;
};

int main(int argc, char** argv) {
    cxxopts::Options options("TraceIt - Wormhole Renderer",
                             "CLI application for rendering a sequence of wormhole images.");

    // clang-format off
    options.add_options()("w,width", "image width", cxxopts::value<uint32_t>()->default_value("640"))(
        "h,height", "image height", cxxopts::value<uint32_t>()->default_value("480"))(
        "d,distance", "distance from wormhole center", cxxopts::value<float>()->default_value("10.0"))(
        "a,wormhole_length", "wormhole length a", cxxopts::value<float>()->default_value("0.1"))(
        "r,wormhole_throat_radius", "wormhole throat radius", cxxopts::value<float>()->default_value("2.0"))(
        "m,wormhole_mass_param", "wormhole mass parameter", cxxopts::value<float>()->default_value("0.2"))(
        "l,lower_sphere_id", "texture id of lower sphere", cxxopts::value<int>()->default_value("0"))(
        "u,upper_sphere_id", "texture id of upper sphere", cxxopts::value<int>()->default_value("2"))(
        "azimuth_velo", "azimuthal velocity of camera", cxxopts::value<float>()->default_value("0"))(
        "radial_velo", "radial velocity of camera towards wormhole", cxxopts::value<float>()->default_value("0.0"))(
        "duration", "animation time [s]", cxxopts::value<float>()->default_value("5.0"))(
        "help", "Print usage");
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    auto cam_pos = glm::vec3(0.f, 0.f, -result["distance"].as<float>());
    auto cam_dir = glm::vec3(0.f, 0.f, 1.f);
    // auto cam_pos = glm::vec3(0.f, -5.f, 0.f);
    // auto cam_dir = normalize(glm::vec3(0.f, 1.f, 0.f));

    auto wormhole_scene =
        SceneSetup(result["width"].as<uint32_t>(), result["height"].as<uint32_t>(), cam_pos, cam_dir,
                   result["wormhole_length"].as<float>(), result["wormhole_throat_radius"].as<float>(),
                   result["wormhole_mass_param"].as<float>(), result["lower_sphere_id"].as<int>(),
                   result["upper_sphere_id"].as<int>());

    wormhole_scene.update(cam_pos, cam_dir);  // initial cam setup

    const float sim_duration_s = result["duration"].as<float>();
    const float sim_time_increment_s = 1 / kOutputFrameRate;
    const float azimuth_velo = result["azimuth_velo"].as<float>();
    const float radial_velo = result["radial_velo"].as<float>();

    auto updateCamPos = [&]() {
        auto cam_pos_sph = cartToSpherical(cam_pos);
        cam_pos_sph.x += radial_velo * sim_time_increment_s;
        cam_pos_sph.z += azimuth_velo * sim_time_increment_s;
        cam_pos = sphericalToCart(cam_pos_sph);
        cam_dir = normalize(-cam_pos);
        wormhole_scene.update(cam_pos, cam_dir);
    };

    // render loop for all camera positions
    bool first_run = true;
    for (float sim_time_s = 0.f; sim_time_s < sim_duration_s; sim_time_s += sim_time_increment_s) {
        TRACEIT_LOG_INFO("Rendering timestep: " << sim_time_s << " / " << sim_duration_s << " for pos: (" << cam_pos.x
                                                << ", " << cam_pos.y << ", " << cam_pos.z << ")");
        wormhole_scene.render();
        updateCamPos();

        if (first_run) {
            first_run = false;
            const auto num_images = static_cast<int>(sim_duration_s / sim_time_increment_s);
            TRACEIT_LOG_INFO("Expected render time for all "
                             << num_images << " images: " << num_images * wormhole_scene.curremtFrameTimeSecs()
                             << " s");
        }
    }

    return 0;
}
