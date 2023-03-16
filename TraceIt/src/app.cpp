// TraceIt Main Application

#define STB_IMAGE_IMPLEMENTATION  // needed once to use stb_image header properly

#include <chrono>
#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "camera.h"
#include "imgui.h"
#include "renderer.h"
#include "scene.h"
#include "ui.h"

using namespace Ui;

class SceneLayer : public Layer {
   public:
    SceneLayer() : m_camera(glm::vec3(0.f, 0.f, -10.f), glm::vec3(0.f, 0.f, 1.f)), m_renderer(m_camera) {
        m_last_stop_time = std::chrono::high_resolution_clock::now();
        m_object_types = m_scene.availableObjectTypes();
        m_cur_obj_selection = m_object_types[0];
        m_cur_lower_sphere_selection = m_sphere_texture_names[0];
        m_cur_upper_sphere_selection = m_sphere_texture_names[2];
    }

    virtual void onUpdate() override { m_camera.updatePose(static_cast<float>(m_frame_time_ms.count()) / 1e3); }

    virtual void onRender() override {
        ImGui::Begin("Scene Explorer");

        if (ImGui::BeginCombo("Object", m_cur_obj_selection.c_str())) {
            for (const auto& object_type : m_object_types) {
                bool is_selected = (m_cur_obj_selection.c_str() == object_type.c_str());
                if (ImGui::Selectable(object_type.c_str(), is_selected)) {
                    m_cur_obj_selection = object_type;
                }
                if (is_selected) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (ImGui::Button("+")) {
            m_scene.addObject(m_cur_obj_selection);
        }

        ImGui::Separator();

        if (ImGui::TreeNodeEx("Lights", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
            for (auto light : m_scene.lights()) {
                if (ImGui::TreeNode(light->name.c_str())) {
                    if (auto point_light = std::dynamic_pointer_cast<PointLight>(light)) {
                        drawAndUpdateObjectSettings(*point_light);
                    }
                    if (ImGui::Button("Remove")) {
                        m_scene.eraseLightSource(light);
                        ImGui::TreePop();
                        break;
                    }
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }

        ImGui::Separator();

        if (ImGui::TreeNodeEx("Hittable Objects", ImGuiTreeNodeFlags_DefaultOpen | ImGuiTreeNodeFlags_Framed)) {
            for (auto obj_variant : m_scene.objects()) {
                bool is_current_element = false;
                std::visit(
                    [&](auto&& obj) {
                        if (ImGui::TreeNode(obj.name.c_str())) {
                            drawAndUpdateObjectSettings(obj);
                            is_current_element = true;
                        }
                    },
                    *obj_variant);

                // easier to handle erase + treepop outside of the visit lambda
                if (is_current_element) {
                    if (ImGui::Button("Remove")) {
                        m_scene.eraseHittableObject(obj_variant);
                        ImGui::TreePop();
                        break;
                    }
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }

        ImGui::End();

        // Image output
        ImGui::Begin("Renderer");

        const auto frame_ms = static_cast<float>(m_frame_time_ms.count());
        ImGui::Text("Framerate: %.1f ms/frame (%.1f fps)", frame_ms, 1e3 / frame_ms);
        const auto cam_pos = m_camera.position();
        ImGui::Text("Camera position: (%.1f, %.1f, %.1f)", cam_pos.x, cam_pos.y, cam_pos.z);

        ImGui::Checkbox("Auto rendering", &m_auto_render);

        if (ImGui::Button("Render") || m_auto_render) {
            render();
        }

        ImGui::Separator();

        const auto region_avail = ImGui::GetContentRegionAvail();
        m_render_image_width = region_avail.x;
        m_render_image_height = region_avail.y;

        const auto image = m_renderer.image();
        if (image) {
            ImGui::Image((ImTextureID)image->descrSet(),
                         ImVec2(static_cast<float>(image->width()), static_cast<float>(image->height())), ImVec2(0, 1),
                         ImVec2(1, 0));
        }

        ImGui::End();
    }

   private:
    void render() {
        const auto time_since_last_render_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - m_last_stop_time);
        if (time_since_last_render_ms.count() < 10) {  // cap to 100 fps
            return;
        }
        const auto start_time = m_auto_render ? m_last_stop_time : std::chrono::high_resolution_clock::now();
        m_camera.refresh(m_render_image_width, m_render_image_height);  // -> on viewport change only?
        m_renderer.refresh(m_render_image_width, m_render_image_height);
        m_renderer.render(m_scene);
        m_last_stop_time = std::chrono::high_resolution_clock::now();
        m_frame_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(m_last_stop_time - start_time);
    }

    void drawAndUpdateObjectSettings(PointLight& obj) const {
        ImGui::Text("Position");
        ImGui::SliderFloat("x", &obj.position.x, -100.f, 100.0f, "%1.f");
        ImGui::SliderFloat("y", &obj.position.y, -100.f, 100.0f, "%1.f");
        ImGui::SliderFloat("z", &obj.position.z, -100.f, 100.0f, "%1.f");
        float color[4] = {obj.color[0], obj.color[1], obj.color[2], 1.f};
        ImGui::ColorEdit3("color", color);
        obj.color = glm::vec4(color[0], color[1], color[2], 1.f);
        ImGui::SliderFloat("intensity", &obj.intensity, 0.f, 1.0f, "%.1f");
    }

    void drawAndUpdateObjectSettings(Plane& obj) const {
        ImGui::Text("Position");
        ImGui::SliderFloat("z", &obj.position.z, -50.f, 50.0f, "%.1f");
    }

    void drawAndUpdateObjectSettings(Cube& obj) const {
        ImGui::Text("Position");
        ImGui::SliderFloat("x", &obj.position.x, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("y", &obj.position.y, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("z", &obj.position.z, -50.f, 50.0f, "%.1f");
    }

    void drawAndUpdateObjectSettings(Sphere& obj) const {
        ImGui::Text("Position");
        ImGui::SliderFloat("x", &obj.position.x, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("y", &obj.position.y, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("z", &obj.position.z, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("r", &obj.radius, 0.f, 10.0f, "%.1f");
        float color[4] = {obj.color[0], obj.color[1], obj.color[2], 1.f};
        ImGui::ColorEdit3("color", color);
        obj.color = glm::vec4(color[0], color[1], color[2], 1.f);
    }

    void drawAndUpdateObjectSettings(ExtendedEllisWormhole& obj) {
        auto spherTextureComboBox = [&](const char* label, std::string& selection_name,
                                        ExtendedEllisWormhole::CelestialSphereType& sphere_type) {
            if (ImGui::BeginCombo(label, selection_name.c_str())) {
                for (int i = 0; i < m_sphere_texture_names.size(); ++i) {
                    const auto* texture_name = m_sphere_texture_names[i];
                    bool is_selected = (selection_name.c_str() == texture_name);
                    if (ImGui::Selectable(texture_name, is_selected)) {
                        selection_name = std::string(texture_name);
                        sphere_type = static_cast<ExtendedEllisWormhole::CelestialSphereType>(i);
                        obj.loadImages();
                    }
                    if (is_selected) ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
        };

        spherTextureComboBox("Lower Sphere", m_cur_lower_sphere_selection, obj.lower_celestial_sphere);
        spherTextureComboBox("Upper Sphere", m_cur_upper_sphere_selection, obj.upper_celestial_sphere);

        ImGui::Checkbox("Reuse eq. plane ray paths", &obj.compute_rays_in_eq_plane);

        ImGui::SliderFloat("wormhole length", &obj.a, 0.f, 10.0f, "%.1f");
        ImGui::SliderFloat("throat radius", &obj.rho, 0.f, 10.0f, "%.1f");
        ImGui::SliderFloat("M", &obj.M, 0.f, 10.0f, "%.1f");
    }

   private:
    const std::vector<const char*> m_sphere_texture_names = {"Interstellar's Saturn Site",
                                                             "Galaxy String",
                                                             "Interstellar's Far Galaxy",
                                                             "Bluegrey Nebula",
                                                             "Brown Nebula",
                                                             "Green Nebula",
                                                             "Greenish-Brown Nebula",
                                                             "Smokey Nebula"};

    std::chrono::milliseconds m_frame_time_ms{0};
    std::chrono::steady_clock::time_point m_last_stop_time;
    std::vector<std::string> m_object_types;
    std::string m_cur_obj_selection;
    std::string m_cur_lower_sphere_selection;
    std::string m_cur_upper_sphere_selection;
    Scene m_scene;
    Renderer m_renderer;
    Camera m_camera;
    bool m_auto_render = false;
    uint32_t m_render_image_width = 0;
    uint32_t m_render_image_height = 0;
};

int main(int, char**) {
    Config uiconfig{"TraceIt Studio", 1280, 720};
    App app(uiconfig);
    app.addLayer<SceneLayer>();

    app.run();

    return 0;
}
