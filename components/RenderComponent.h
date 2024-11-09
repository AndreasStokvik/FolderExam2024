#pragma once
#include <memory>
#include <string>
#include "../../QengineProject/Model.h"

struct RenderComponent {
    std::shared_ptr<Model> model;

    RenderComponent() : model(nullptr) {}

    RenderComponent(const std::shared_ptr<Model>& modelPtr)
        : model(modelPtr) {}
};