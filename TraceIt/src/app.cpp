// TraceIt Main Application

#include <string>
#include <vector>

#include "imgui.h"
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
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }

        ImGui::Separator();

        ImGui::End();
    }

   private:
    std::vector<std::string> m_object_types;
    std::string m_cur_obj_selection;
    Scene m_scene;
};

int main(int, char**) {
    Config uiconfig{"TraceIt Studio", 1280, 720};
    App app(uiconfig);
    app.addLayer<SceneLayer>();

    app.run();

    return 0;
}
