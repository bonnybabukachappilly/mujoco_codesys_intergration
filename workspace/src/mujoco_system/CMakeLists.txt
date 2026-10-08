cmake_minimum_required(VERSION 3.8)
project(mujoco_system)

find_package(ament_cmake REQUIRED)
find_package(dobot_description REQUIRED)
find_package(eoat_description REQUIRED)

get_filename_component(DOBOT_SHARE "${dobot_description_DIR}/.." ABSOLUTE)
install(
  DIRECTORY ${DOBOT_SHARE}/meshes/cr5/
  DESTINATION share/${PROJECT_NAME}/models/robots/dobot/cr5/meshes
)

get_filename_component(EOAT_SHARE "${eoat_description_DIR}/.." ABSOLUTE)
install(
  DIRECTORY ${EOAT_SHARE}/meshes/pgc/pgc_50_35/
  DESTINATION share/${PROJECT_NAME}/models/eoat/pgc/pgc_50_35/meshes
)

install(
  DIRECTORY assets config models scenes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
