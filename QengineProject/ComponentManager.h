#pragma once
#include <unordered_map>

template <typename T>
class ComponentManager {
public:
    void addComponent(int entityId, T component) {
        components[entityId] = component;
    }

    T& getComponent(int entityId) {
        return components.at(entityId);
    }

    bool hasComponent(int entityId) const {
        return components.find(entityId) != components.end();
    }

private:
    std::unordered_map<int, T> components;
};