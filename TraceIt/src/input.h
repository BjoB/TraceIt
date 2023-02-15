#pragma once

#include <imgui.h>

namespace input {

bool forwardPressed() { return ImGui::IsKeyPressed(ImGuiKey_W); }

bool backwardPressed() { return ImGui::IsKeyPressed(ImGuiKey_S); }

}  // namespace input
