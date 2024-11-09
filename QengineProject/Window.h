#ifndef WINDOW_H
#define WINDOW_H
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>

#include "Camera.h"

class Window {
public:
    int width, height;
    std::shared_ptr<Camera> camera;
    GLFWwindow* window;

    Window(int width, int height, const std::string& title, const std::shared_ptr<Camera>& camera);
    ~Window();

    GLFWwindow* getWindow() { return window; };
    void swapBuffers();
    bool shouldClose();
    bool isKeyPressed(int key);
    float getAspectRatio() const;
    void toggleCursor();

    // Set the mouse callback
    void setMouseCallback(GLFWcursorposfun callback) {
        glfwSetCursorPosCallback(window, callback);
    }
    void setUserPointer(void* pointer) {
        glfwSetWindowUserPointer(window, pointer);
    }

    void setSize(int newWidth, int newHeight);

    static void framebufferSizeCallback(GLFWwindow* window, int width, int height);

private:
    bool cursorHidden;

    void initGLFW();
};

#endif WINDOW_H