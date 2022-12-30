// TraceIt Main Application

#include "imgui.h"
#include "ui.h"

using namespace Ui;

class SceneExplorer : public Layer {
   public:
    SceneExplorer() {}

    virtual void onUpdate() override {}

    virtual void onRender() override {
        ImGui::Begin("Scene Explorer");
        ImGui::Text("tbd");
        ImGui::End();
    }

   private:
};

int main(int, char**) {
    Config uiconfig{"TraceIt Studio", 1280, 720};
    App app(uiconfig);
    app.addLayer<SceneExplorer>();

    app.run();

    return 0;
}
