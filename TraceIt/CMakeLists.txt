# TraceIt Studio cmake project

# Dependencies -------------------------------------------------------

## ImGui

set(imgui_SRC_DIR "3rdparty/imgui-1.89.1")

add_library(imgui)

target_include_directories(imgui PUBLIC
    ${imgui_SRC_DIR}/
    ${imgui_SRC_DIR}/backends
)

target_sources(imgui PRIVATE
    ${imgui_SRC_DIR}/imgui.h
    ${imgui_SRC_DIR}/imgui_internal.h
    ${imgui_SRC_DIR}/imgui.cpp
    ${imgui_SRC_DIR}/imgui_demo.cpp
    ${imgui_SRC_DIR}/imgui_draw.cpp
    ${imgui_SRC_DIR}/imgui_tables.cpp
    ${imgui_SRC_DIR}/imgui_widgets.cpp
)

## glfw

set(glfw_SRC_DIR "3rdparty/glfw-3.3.8")

add_library(glfw SHARED IMPORTED)

target_include_directories(glfw INTERFACE "${glfw_SRC_DIR}/include")

set_target_properties(glfw PROPERTIES
    IMPORTED_LOCATION "${glfw_SRC_DIR}/lib-vc2022/glfw3.dll"
    IMPORTED_IMPLIB "${glfw_SRC_DIR}/lib-vc2022/glfw3.lib"
)

## Vulkan

find_package(Vulkan REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVK_PROTOTYPES")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_PROTOTYPES")

# Build --------------------------------------------------------------

add_executable(studio "src/app.cpp" "src/app.h")

target_link_libraries(studio
    PRIVATE
        glfw
        imgui
        Vulkan::Vulkan
)
