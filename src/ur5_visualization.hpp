#pragma once

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <vector>

namespace ur5vis {

struct MouseState {
  bool left = false;
  bool middle = false;
  bool right = false;
  double last_x = 0.0;
  double last_y = 0.0;
};

inline void InitializeCamera(const mjModel* model, mjvCamera* camera) {
  mjv_defaultFreeCamera(model, camera);
  camera->distance = 1.8;
  camera->azimuth = 145.0;
  camera->elevation = -25.0;
  camera->lookat[0] = -0.25;
  camera->lookat[1] = 0.05;
  camera->lookat[2] = 0.30;
}

inline void ApplyRobotGeometryMode(mjvOption* option, bool show_collision_model) {
  constexpr int kRobotVisualGroup = 2;
  constexpr int kRobotCollisionGroup = 3;
  option->geomgroup[kRobotVisualGroup] = show_collision_model ? 0 : 1;
  option->geomgroup[kRobotCollisionGroup] = show_collision_model ? 1 : 0;
}

inline void AddToolPathMarkers(mjvScene* scene, const std::vector<std::array<double, 3>>& tool_path,
                               const std::array<double, 3>* current_point) {
  constexpr mjtNum kIdentity[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  constexpr float kPathRgba[4] = {0.1F, 0.65F, 1.0F, 0.38F};
  constexpr float kCurrentRgba[4] = {1.0F, 0.85F, 0.1F, 1.0F};

  const std::size_t stride = std::max<std::size_t>(1, tool_path.size() / 120);
  for (std::size_t i = 0; i < tool_path.size(); i += stride) {
    if (scene->ngeom >= scene->maxgeom) {
      return;
    }
    const auto& point = tool_path[i];
    const mjtNum size[3] = {0.008, 0.0, 0.0};
    const mjtNum pos[3] = {point[0], point[1], point[2]};
    mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_SPHERE, size, pos, kIdentity, kPathRgba);
    scene->geoms[scene->ngeom].category = mjCAT_DECOR;
    ++scene->ngeom;
  }

  if (current_point != nullptr && scene->ngeom < scene->maxgeom) {
    const mjtNum size[3] = {0.018, 0.0, 0.0};
    const mjtNum pos[3] = {(*current_point)[0], (*current_point)[1], (*current_point)[2]};
    mjv_initGeom(&scene->geoms[scene->ngeom], mjGEOM_SPHERE, size, pos, kIdentity, kCurrentRgba);
    scene->geoms[scene->ngeom].category = mjCAT_DECOR;
    ++scene->ngeom;
  }
}

inline void AddGoalGhostGeoms(mjvScene* scene,
                              const mjModel* model,
                              const mjData* ghost_data,
                              const std::vector<int>& ghost_geom_ids) {
  constexpr float kGhostRgba[4] = {0.0F, 1.0F, 0.55F, 0.28F};
  for (const int geom_id : ghost_geom_ids) {
    if (scene->ngeom >= scene->maxgeom) {
      return;
    }
    mjvGeom& geom = scene->geoms[scene->ngeom];
    mjv_initGeom(&geom,
                 static_cast<mjtGeom>(model->geom_type[geom_id]),
                 &model->geom_size[3 * geom_id],
                 ghost_data->geom_xpos + 3 * geom_id,
                 ghost_data->geom_xmat + 9 * geom_id,
                 kGhostRgba);
    geom.category = mjCAT_DECOR;
    geom.objtype = mjOBJ_GEOM;
    geom.objid = geom_id;
    geom.segid = -1;
    std::strncpy(geom.label, "goal", sizeof(geom.label) - 1);
    geom.label[sizeof(geom.label) - 1] = '\0';
    ++scene->ngeom;
  }
}

class InteractiveCamera {
 public:
  void Attach(GLFWwindow* window, mjModel* model, mjvScene* scene, mjvCamera* camera) {
    window_ = window;
    model_ = model;
    scene_ = scene;
    camera_ = camera;
    glfwSetWindowUserPointer(window_, this);
    glfwSetMouseButtonCallback(window_, &InteractiveCamera::MouseButtonCallback);
    glfwSetCursorPosCallback(window_, &InteractiveCamera::MouseMoveCallback);
    glfwSetScrollCallback(window_, &InteractiveCamera::ScrollCallback);
  }

 private:
  static InteractiveCamera* FromWindow(GLFWwindow* window) {
    return static_cast<InteractiveCamera*>(glfwGetWindowUserPointer(window));
  }

  static void MouseButtonCallback(GLFWwindow* window, int, int, int) {
    InteractiveCamera* self = FromWindow(window);
    if (self == nullptr) {
      return;
    }
    self->mouse_.left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    self->mouse_.middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
    self->mouse_.right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
    glfwGetCursorPos(window, &self->mouse_.last_x, &self->mouse_.last_y);
  }

  static void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos) {
    InteractiveCamera* self = FromWindow(window);
    if (self == nullptr || (!self->mouse_.left && !self->mouse_.middle && !self->mouse_.right)) {
      return;
    }

    const double dx = xpos - self->mouse_.last_x;
    const double dy = ypos - self->mouse_.last_y;
    self->mouse_.last_x = xpos;
    self->mouse_.last_y = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0 || self->model_ == nullptr || self->scene_ == nullptr ||
        self->camera_ == nullptr) {
      return;
    }

    const bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;

    mjtMouse action = mjMOUSE_ZOOM;
    if (self->mouse_.right) {
      action = shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (self->mouse_.left) {
      action = shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }

    mjv_moveCamera(self->model_, action, dx / height, dy / height, self->scene_, self->camera_);
  }

  static void ScrollCallback(GLFWwindow* window, double, double yoffset) {
    InteractiveCamera* self = FromWindow(window);
    if (self == nullptr || self->model_ == nullptr || self->scene_ == nullptr ||
        self->camera_ == nullptr) {
      return;
    }
    mjv_moveCamera(self->model_, mjMOUSE_ZOOM, 0.0, -0.05 * yoffset, self->scene_, self->camera_);
  }

  GLFWwindow* window_ = nullptr;
  mjModel* model_ = nullptr;
  mjvScene* scene_ = nullptr;
  mjvCamera* camera_ = nullptr;
  MouseState mouse_;
};

}  // namespace ur5vis
