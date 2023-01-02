#pragma once

#include "log.h"

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <iostream>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

enum ObjectType {
    kPlane,
    kCube,
    kSphere,
    kLastObj,
};

enum MaterialType { Lambertian, Metal };

struct Object {
    virtual ~Object() = default;

    // virtual const char* objTypeName() const = 0;
    // virtual void render() = 0;

    std::string name;
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

    template <class ObjectT>
    void addObject(const std::string& obj_name = "New Object") {
        static_assert(std::is_base_of_v<Object, ObjectT>, "Scene objects need to derive from Object struct!");
        m_objects.push_back(std::make_shared<ObjectT>());
        m_objects.back()->name = obj_name;
    }

    bool addObject(const std::string& object_type) {
        const auto next_id = static_cast<int>(m_objects.size()) + 1;
        const auto next_id_str = std::to_string(next_id);

        if (object_type == "Plane")
            addObject<Plane>(std::string("Plane") + next_id_str);
        else if (object_type == "Cube")
            addObject<Cube>(std::string("Cube") + next_id_str);
        else if (object_type == "Sphere")
            addObject<Sphere>("Sphere" + next_id_str);
        else {
            TRACEIT_LOG_ERROR("Unknown object type " << object_type << ".");
            return false;
        }
        return true;
    }

    auto& objects() const { return m_objects; }

    auto availableObjectTypes() const {
        std::vector<std::string> obj_names;
        for (auto& [_, val] : m_available_objects_map) {
            obj_names.push_back(std::string(val));
        }
        return obj_names;
    }

   private:
    const std::unordered_map<ObjectType, const char*> m_available_objects_map{
        {ObjectType::kPlane, "Plane"}, {ObjectType::kCube, "Cube"}, {ObjectType::kSphere, "Sphere"}};
    std::vector<std::shared_ptr<Object>> m_objects;
};
