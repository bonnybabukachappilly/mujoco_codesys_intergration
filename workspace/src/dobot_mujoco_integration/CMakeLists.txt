cmake_minimum_required(VERSION 3.8)
project(dobot_mujoco_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(dobot_description REQUIRED)
find_package(eoat_description REQUIRED)

get_filename_component(DOBOT_DESC_SHARE "${dobot_description_DIR}/.." ABSOLUTE)
get_filename_component(GRIPPER_DESC_SHARE "${eoat_description_DIR}/.." ABSOLUTE)

execute_process(
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_cell.py ${CMAKE_CURRENT_SOURCE_DIR}
  RESULT_VARIABLE gen_rc
)

if(NOT gen_rc EQUAL 0)
  message(FATAL_ERROR "generate_cell.py failed")
endif()

install(
  DIRECTORY ${DOBOT_DESC_SHARE}/meshes/cr5/
  DESTINATION share/${PROJECT_NAME}/robots/cr5/meshes
)

install(
  DIRECTORY ${GRIPPER_DESC_SHARE}/meshes/pgc_50_35/
  DESTINATION share/${PROJECT_NAME}/grippers/pgc_50_35/meshes
)

install(
  DIRECTORY robots grippers scenes scripts config urdf
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
