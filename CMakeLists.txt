cmake_minimum_required(VERSION 3.28)

project(ur5_mujoco_ompl
  VERSION 0.1.0
  DESCRIPTION "Local MuJoCo + OMPL motion planning lab for a UR5e manipulator"
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

set(MUJOCO_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_SIMULATE OFF CACHE BOOL "" FORCE)
set(MUJOCO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MUJOCO_TEST_PYTHON_UTIL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  mujoco
  URL https://github.com/google-deepmind/mujoco/archive/refs/tags/3.10.0.tar.gz
)
FetchContent_MakeAvailable(mujoco)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
if(NOT TARGET glfw)
  FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
  )
  FetchContent_MakeAvailable(glfw)
endif()

FetchContent_Declare(
  mujoco_menagerie
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco_menagerie.git
  GIT_TAG accb6df40a9a1d1e49eff88157f6818b63a49335
  SOURCE_SUBDIR cmake-do-not-add
)
FetchContent_MakeAvailable(mujoco_menagerie)

set(UR5E_MENAGERIE_DIR "${mujoco_menagerie_SOURCE_DIR}/universal_robots_ur5e")
set(UR5E_MODEL_XML_SOURCE "${UR5E_MENAGERIE_DIR}/ur5e.xml")
set(UR5E_MODEL_XML "${CMAKE_CURRENT_BINARY_DIR}/scenes/ur5e_absolute_assets.xml")
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/scenes")
file(READ "${UR5E_MODEL_XML_SOURCE}" UR5E_MODEL_XML_CONTENTS)
string(
  REPLACE
    "meshdir=\"assets\""
    "meshdir=\"${UR5E_MENAGERIE_DIR}/assets\""
    UR5E_MODEL_XML_CONTENTS
    "${UR5E_MODEL_XML_CONTENTS}"
)
file(WRITE "${UR5E_MODEL_XML}" "${UR5E_MODEL_XML_CONTENTS}")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/scenes/ur5e_clutter.xml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/scenes/ur5e_clutter.xml"
  @ONLY
)

FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
  OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(Eigen3)

set(BOOST_INCLUDE_LIBRARIES serialization program_options graph math CACHE STRING "" FORCE)
set(BOOST_ENABLE_CMAKE ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  Boost
  URL https://github.com/boostorg/boost/releases/download/boost-1.91.0-1/boost-1.91.0-1-cmake.tar.xz
  URL_HASH SHA256=cc5dc5006ecbdf0051f90979be31b4eee5987d9ae14ae9fb9c03cfa43fa3cdad
  OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(Boost)
file(GLOB UR5_BOOST_MODULE_INCLUDES CONFIGURE_DEPENDS "${boost_SOURCE_DIR}/libs/*/include")
include_directories(SYSTEM ${UR5_BOOST_MODULE_INCLUDES})

set(OMPL_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_PYTHON_BINDINGS OFF CACHE BOOL "" FORCE)
set(OMPL_BUILD_VAMP OFF CACHE BOOL "" FORCE)
set(OMPL_REGISTRATION OFF CACHE BOOL "" FORCE)
set(SKBUILD ON CACHE BOOL "" FORCE)
FetchContent_Declare(
  ompl
  URL https://github.com/ompl/ompl/releases/download/2.0.1/ompl-2.0.1.tar.gz
  URL_HASH SHA256=62401cfd65de7b7fec361586aaa7ff9d5d54b685c8807f5e5501ba3f929699b0
)
FetchContent_MakeAvailable(ompl)

if(TARGET mujoco::mujoco)
  set(UR5_MUJOCO_TARGET mujoco::mujoco)
elseif(TARGET mujoco)
  set(UR5_MUJOCO_TARGET mujoco)
else()
  message(FATAL_ERROR "Could not find MuJoCo CMake target")
endif()

if(TARGET ompl)
  set(UR5_OMPL_TARGET ompl)
elseif(TARGET ompl::ompl)
  set(UR5_OMPL_TARGET ompl::ompl)
else()
  message(FATAL_ERROR "Could not find OMPL CMake target")
endif()

add_executable(ur5_clutter_plan
  src/ur5_clutter_plan.cpp
)
target_compile_features(ur5_clutter_plan PRIVATE cxx_std_23)
target_link_libraries(ur5_clutter_plan
  PRIVATE
    ${UR5_MUJOCO_TARGET}
    ${UR5_OMPL_TARGET}
    glfw
)
target_compile_definitions(ur5_clutter_plan
  PRIVATE
    UR5_MUJOCO_OMPL_DEFAULT_SCENE="${CMAKE_CURRENT_BINARY_DIR}/scenes/ur5e_clutter.xml"
)

add_executable(ur5_path_replay
  src/ur5_path_replay.cpp
)
target_compile_features(ur5_path_replay PRIVATE cxx_std_23)
target_link_libraries(ur5_path_replay
  PRIVATE
    ${UR5_MUJOCO_TARGET}
    glfw
)
target_compile_definitions(ur5_path_replay
  PRIVATE
    UR5_MUJOCO_OMPL_DEFAULT_SCENE="${CMAKE_CURRENT_BINARY_DIR}/scenes/ur5e_clutter.xml"
)

add_executable(ur5_path_diagnose
  src/ur5_path_diagnose.cpp
)
target_compile_features(ur5_path_diagnose PRIVATE cxx_std_23)
target_link_libraries(ur5_path_diagnose
  PRIVATE
    ${UR5_MUJOCO_TARGET}
)
target_compile_definitions(ur5_path_diagnose
  PRIVATE
    UR5_MUJOCO_OMPL_DEFAULT_SCENE="${CMAKE_CURRENT_BINARY_DIR}/scenes/ur5e_clutter.xml"
)

if(MSVC)
  target_compile_options(ur5_clutter_plan PRIVATE /W4)
  target_compile_options(ur5_path_replay PRIVATE /W4)
  target_compile_options(ur5_path_diagnose PRIVATE /W4)
else()
  target_compile_options(ur5_clutter_plan PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(ur5_path_replay PRIVATE -Wall -Wextra -Wpedantic)
  target_compile_options(ur5_path_diagnose PRIVATE -Wall -Wextra -Wpedantic)
endif()
