#pragma once
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <string>
#include <memory>

#include "Window.h"

class ImGuiManager
{
public:
	ImGuiManager(const std::shared_ptr<Window>& window);
	void BasicText(const std::string& stringA, const std::string& stringB);
	void BasicCheckbox(const std::string& windowTitle, const std::string& label, bool& checkboxState);
	void DemoWindow(const std::string& windowTitle);
	void shutdown();
};

