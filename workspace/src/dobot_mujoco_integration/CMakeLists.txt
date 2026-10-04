cmake_minimum_required(VERSION 3.8)
project(dobot_mujoco_integration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dobot_description REQUIRED)
find_package(eoat_description REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Fail early with a clear message if the chosen interpreter has no PyYAML.
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import yaml"
  RESULT_VARIABLE yaml_rc OUTPUT_QUIET ERROR_QUIET)
if(NOT yaml_rc EQUAL 0)
  message(FATAL_ERROR
    "PyYAML missing for ${Python3_EXECUTABLE}. "
    "Install python3-yaml or pass -DPython3_EXECUTABLE=/usr/bin/python3")
endif()

get_filename_component(DOBOT_DESC_SHARE "${dobot_description_DIR}/.." ABSOLUTE)
get_filename_component(GRIPPER_DESC_SHARE "${eoat_description_DIR}/.." ABSOLUTE)

# ---- generation (build time, dependency-tracked) ----
set(GEN_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_cell.py)
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GEN_STAMP ${GEN_DIR}/.generated)

add_custom_command(
  OUTPUT ${GEN_STAMP}
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${GEN_SCRIPT}
  --src ${CMAKE_CURRENT_SOURCE_DIR} --out ${GEN_DIR} --stamp ${GEN_STAMP}
  DEPENDS ${GEN_SCRIPT}
  ${CMAKE_CURRENT_SOURCE_DIR}/config/cell.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/config/grippers.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/robots/cr5/robot.xml.in
  COMMENT "Generating cell models from cell.yaml"
  VERBATIM)
add_custom_target(generate_cell ALL DEPENDS ${GEN_STAMP})

# ---- install ----
install(DIRECTORY ${DOBOT_DESC_SHARE}/meshes/cr5/
  DESTINATION share/${PROJECT_NAME}/robots/cr5/meshes)
install(DIRECTORY ${GRIPPER_DESC_SHARE}/meshes/pgc_50_35/
  DESTINATION share/${PROJECT_NAME}/grippers/pgc_50_35/meshes)

# generated files merge into the same share/ layout as the static ones
install(DIRECTORY ${GEN_DIR}/
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".generated" EXCLUDE)
install(DIRECTORY robots grippers config
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
