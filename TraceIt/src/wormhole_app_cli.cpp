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
#include "renderer.h"
#include "scene.h"

constexpr float kOutputFrameRate = 30.f;

class SceneSetup {
   public:
    SceneSetup(uint32_t image_width, uint32_t image_height, glm::vec3 cam_pos, glm::vec3 cam_dir,
               float wormhole_length_a, float wormhole_throat_radius_rho, float wormhole_mass_param)
        : m_render_image_width(image_width),
          m_render_image_height(image_height),
          m_camera(cam_pos, cam_dir),
          m_renderer(m_camera) {
        m_scene.addObject("Wormhole");
        m_wormhole_obj = &std::get<ExtendedEllisWormhole>(*m_scene.objects().back());
        m_wormhole_obj->a = wormhole_length_a;
        m_wormhole_obj->rho = wormhole_throat_radius_rho;
        m_wormhole_obj->M = wormhole_mass_param;
    }

    void update() {
        const auto new_pos = glm::vec3();
        const auto new_dir = glm::vec3();
        m_camera.updatePose(new_pos, new_dir);
    }

    void render() {
        const auto cam_pos = m_camera.position();

        const auto start_time = std::chrono::high_resolution_clock::now();
        m_camera.refresh(m_render_image_width, m_render_image_height);
        m_renderer.refresh(m_render_image_width, m_render_image_height);
        m_renderer.render(m_scene);
        m_frame_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - start_time);

        const auto image = m_renderer.image();
        if (image) {
            const auto img_filename = std::string("wh_") + std::to_string(m_cur_img_index) + std::string(".png");
            // stbi_write_png(img_filename.c_str(), m_render_image_width, m_render_image_height, 3, image.get(),
            //                m_render_image_width * 3);
            ++m_cur_img_index;
        }
    }

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
        "azimuth_velo", "azimuthal velocity of camera", cxxopts::value<float>()->default_value("0"))(
        "radial_velo", "radial velocity of camera", cxxopts::value<float>()->default_value("0"))(
        "duration", "animation time [s]", cxxopts::value<float>()->default_value("5.0"))(
        "help", "Print usage");
    // clang-format on

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    const auto neg_dist_from_wh = -result["distance"].as<float>();

    auto wormhole_scene = SceneSetup(
        result["width"].as<uint32_t>(), result["height"].as<uint32_t>(), glm::vec3(0.f, 0.f, neg_dist_from_wh),
        glm::vec3(0.f, 0.f, 1.f), result["wormhole_length"].as<float>(), result["wormhole_throat_radius"].as<float>(),
        result["wormhole_mass_param"].as<float>());

    return 0;
}
