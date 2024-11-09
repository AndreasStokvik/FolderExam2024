#include "ImGuiManager.h"

ImGuiManager::ImGuiManager(const std::shared_ptr<Window>& window)
{
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window->getWindow(), true);
    ImGui_ImplOpenGL3_Init("#version 330");

    glfwSetMouseButtonCallback(window->getWindow(), ImGui_ImplGlfw_MouseButtonCallback);
    glfwSetScrollCallback(window->getWindow(), ImGui_ImplGlfw_ScrollCallback);
    glfwSetKeyCallback(window->getWindow(), ImGui_ImplGlfw_KeyCallback);
    glfwSetCharCallback(window->getWindow(), ImGui_ImplGlfw_CharCallback);

    assert(ImGui::GetCurrentContext() != nullptr);
}

void ImGuiManager::BasicText(const std::string& stringA, const std::string& stringB)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin(stringA.c_str());
    ImGui::Text("%s", stringB.c_str());
    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::BasicCheckbox(const std::string& windowTitle, const std::string& label, bool& checkboxState)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin(windowTitle.c_str());

    ImGui::Checkbox(label.c_str(), &checkboxState);

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::DemoWindow(const std::string& windowTitle)
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::Begin(windowTitle.c_str());

    bool showDemo = true;
    ImGui::ShowDemoWindow(&showDemo);

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::shutdown()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}
