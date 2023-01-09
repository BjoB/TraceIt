// TraceIt Main Application

#include <string>
#include <vector>

#include "imgui.h"
#include "renderer.h"
#include "scene.h"
#include "ui.h"

using namespace Ui;

class SceneLayer : public Layer {
   public:
    SceneLayer() {
        m_object_types = m_scene.availableObjectTypes();
        m_cur_obj_selection = m_object_types[0];
    }

    virtual void onUpdate() override {}

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

        bool add_object_pressed = ImGui::Button("+");
        if (add_object_pressed) {
            m_scene.addObject(m_cur_obj_selection);
        }

        ImGui::Separator();
        if (ImGui::TreeNode("Scene Objects")) {
            for (const auto& obj : m_scene.objects()) {
                if (ImGui::TreeNode(obj->name.c_str())) {
                    drawObjectSettings(*obj);
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }

        ImGui::End();

        // Image output
        ImGui::Begin("Renderer");

        if (ImGui::Button("Render")) {
            render();
        }

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

        //render();
    }

   private:
    void render() {
        m_renderer.refresh(m_render_image_width, m_render_image_height);
        m_renderer.render(m_scene);
    }

    void drawObjectSettings(Object& obj) {
        ImGui::Separator();
        ImGui::Text("Object Settings");
        if (obj.name.find("Plane") != std::string::npos) {
            drawObjectSettings(dynamic_cast<Plane&>(obj));
        } else if (obj.name.find("Cube") != std::string::npos) {
            drawObjectSettings(dynamic_cast<Cube&>(obj));
        } else if (obj.name.find("Sphere") != std::string::npos) {
            drawObjectSettings(dynamic_cast<Sphere&>(obj));
        }
        ImGui::Separator();
    }

    void drawObjectSettings(Plane& obj) {
        ImGui::Text("Position");
        ImGui::SliderFloat("z", &obj.position.z, -50.f, 50.0f, "%.1f");
    }

    void drawObjectSettings(Cube& obj) {
        ImGui::Text("Position");
        ImGui::SliderFloat("x", &obj.position.x, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("y", &obj.position.y, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("z", &obj.position.z, -50.f, 50.0f, "%.1f");
    }

    void drawObjectSettings(Sphere& obj) {
        ImGui::Text("Position");
        ImGui::SliderFloat("x", &obj.position.x, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("y", &obj.position.y, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("z", &obj.position.z, -50.f, 50.0f, "%.1f");
        ImGui::SliderFloat("r", &obj.radius, 0.f, 10.0f, "%.1f");
    }

    std::vector<std::string> m_object_types;
    std::string m_cur_obj_selection;
    Scene m_scene;
    Renderer m_renderer;
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
