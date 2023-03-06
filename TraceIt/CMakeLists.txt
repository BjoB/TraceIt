# TraceIt Studio cmake project

# Dependencies -------------------------------------------------------

## glfw

set(glfw_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/3rdparty/glfw-3.3.8")

add_library(glfw SHARED IMPORTED)

target_include_directories(glfw INTERFACE "${glfw_SRC_DIR}/include")

set_target_properties(glfw PROPERTIES
    IMPORTED_LOCATION "${glfw_SRC_DIR}/lib-vc2022/glfw3.dll"
    IMPORTED_IMPLIB "${glfw_SRC_DIR}/lib-vc2022/glfw3.lib"
)

## stb

add_library(stb INTERFACE)

target_include_directories(stb INTERFACE "${CMAKE_CURRENT_LIST_DIR}/3rdparty/stb")

## Vulkan

find_package(Vulkan REQUIRED)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVK_PROTOTYPES")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVK_PROTOTYPES")

## ImGui

set(imgui_SRC_DIR "${CMAKE_CURRENT_LIST_DIR}/3rdparty/imgui-1.89.1")

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
    ${imgui_SRC_DIR}/backends/imgui_impl_glfw.h
    ${imgui_SRC_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SRC_DIR}/backends/imgui_impl_vulkan.h
    ${imgui_SRC_DIR}/backends/imgui_impl_vulkan.cpp
 "src/input.h")

target_link_libraries(imgui PRIVATE glfw Vulkan::Vulkan)

# cxxopts

add_library(cxxopts INTERFACE)

target_include_directories(cxxopts INTERFACE "${CMAKE_CURRENT_LIST_DIR}/3rdparty/cxxopts")

# Build TraceIt lib + apps -----------------------------------------------------------

add_library(traceit_lib)

target_sources(traceit_lib
    PUBLIC
        "src/camera.cpp"
        "src/image.cpp"
        "src/renderer.cpp"
        "src/ui.cpp"
)

target_include_directories(traceit_lib
    PUBLIC
        src
)

target_link_libraries(traceit_lib
    PUBLIC
        glfw
        imgui
        Vulkan::Vulkan
        stb
)

target_compile_definitions(traceit_lib
    PUBLIC
        WORMHOLE_1_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/interstellar_wormhole_1750x875.jpg"
        WORMHOLE_2_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/interstellar_wormhole_2_1750x875.jpg"
        WORMHOLE_3_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/interstellar_wormhole_3_1750x875.jpg"
        WORMHOLE_4_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/space_eve_online_8K_bluegrey_nebula.jpg"
        WORMHOLE_5_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/space_eve_online_8K_brown_nebula.jpg"
        WORMHOLE_6_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/space_eve_online_8K_green_nebula.jpg"
        WORMHOLE_7_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/space_eve_online_8K_nebula.jpg"
        WORMHOLE_8_IMG_SPHERE_PATH="${CMAKE_CURRENT_LIST_DIR}/data/space_eve_online_8K_smokey_nebula.jpg"
)

# full render GUI

add_executable(traceit_studio
    "src/app.cpp"
)

target_link_libraries(traceit_studio
    PRIVATE
        traceit_lib
)

# CLI-based GR wormhole renderer

add_executable(traceit_cli
    "src/wormhole_app_cli.cpp"
)

target_link_libraries(traceit_cli
    PRIVATE
        traceit_lib
        cxxopts
)
