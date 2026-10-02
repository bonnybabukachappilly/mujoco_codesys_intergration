cmake_minimum_required(VERSION 3.8)
project(dobot_mujoco_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dobot_description REQUIRED)

get_filename_component(DOBOT_DESC_SHARE "${dobot_description_DIR}/.." ABSOLUTE)

install(DIRECTORY robots scenes
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY ${DOBOT_DESC_SHARE}/meshes/cr5/
  DESTINATION share/${PROJECT_NAME}/robots/cr5/meshes)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
