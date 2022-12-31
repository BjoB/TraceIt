#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

enum ObjectType {
    Plane,
    Cube,
    Sphere,
    LastObj,
};

enum MaterialType { Lambertian, Metal };

struct Object {
    virtual ~Object() = default;

    // virtual const char* objTypeName() const = 0;
    // virtual void render() = 0;

    glm::vec3 position;
    glm::vec4 color;
    MaterialType material;
};

struct Plane : Object {};

struct Cube : Object {
    float width;
    float height;
};

struct Sphere : Object {
    float radius;
};

class Scene {
   public:
    Scene() {}

    template <typename ObjectT>
    void addObject() {
        static_assert(std::is_base_of_v<Object, ObjectT>, "Scene objects need to derive from Object struct!");
        m_objects.push_back(std::make_shared<ObjectT>());
    }

    auto availableObjectTypes() const {
        std::vector<std::string> obj_names;
        for (auto& [_, val] : m_available_objects_map) {
            obj_names.push_back(std::string(val));
        }
        return obj_names;
    }
    // findObjectsOfType<ObjectT>()

   private:
    const std::unordered_map<ObjectType, const char*> m_available_objects_map{
        {ObjectType::Plane, "Plane"}, {ObjectType::Cube, "Cube"}, {ObjectType::Sphere, "Sphere"}};
    std::vector<std::shared_ptr<Object>> m_objects;
};
