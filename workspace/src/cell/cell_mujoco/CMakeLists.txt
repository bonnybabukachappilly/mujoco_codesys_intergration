cmake_minimum_required(VERSION 3.8)
project(cell_mujoco)

find_package(ament_cmake REQUIRED)
find_package(cell_description REQUIRED)
find_package(dobot_cr5_description REQUIRED)
find_package(eoat_description REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import yaml"
  RESULT_VARIABLE yaml_rc OUTPUT_QUIET ERROR_QUIET)
if(NOT yaml_rc EQUAL 0)
  message(FATAL_ERROR
    "PyYAML missing for ${Python3_EXECUTABLE}. "
    "Install python3-yaml or pass -DPython3_EXECUTABLE=/usr/bin/python3")
endif()

get_filename_component(CELL_DESC_SHARE  "${cell_description_DIR}/.."       ABSOLUTE)
get_filename_component(ROBOT_DESC_SHARE "${dobot_cr5_description_DIR}/.."  ABSOLUTE)
get_filename_component(TOOLS_DESC_SHARE "${eoat_description_DIR}/.."       ABSOLUTE)

# ---- generation (build time, dependency-tracked) ----
set(GEN_SCRIPT ${CELL_DESC_SHARE}/scripts/generate_cell.py)
set(GEN_DIR    ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GEN_STAMP  ${GEN_DIR}/.generated)

add_custom_command(
  OUTPUT ${GEN_STAMP}
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${GEN_SCRIPT} mujoco
          --cell-share ${CELL_DESC_SHARE}
          --robot-share ${ROBOT_DESC_SHARE}
          --src ${CMAKE_CURRENT_SOURCE_DIR}
          --out ${GEN_DIR} --stamp ${GEN_STAMP}
  DEPENDS ${GEN_SCRIPT}
          ${CELL_DESC_SHARE}/config/cell.yaml
          ${CELL_DESC_SHARE}/config/grippers.yaml
          ${ROBOT_DESC_SHARE}/config/cr5.yaml
          ${ROBOT_DESC_SHARE}/urdf/cr5/cr5.urdf.xacro
          ${ROBOT_DESC_SHARE}/urdf/cr5/cr5_macro.xacro
          ${CMAKE_CURRENT_SOURCE_DIR}/config/cr5_sim.yaml
  COMMENT "Generating MuJoCo models from the CR5 URDF"
  VERBATIM)
add_custom_target(generate_cell_mujoco ALL DEPENDS ${GEN_STAMP})

# ---- install ----
install(DIRECTORY ${ROBOT_DESC_SHARE}/meshes/cr5/
        DESTINATION share/${PROJECT_NAME}/robots/cr5/meshes)
install(DIRECTORY ${TOOLS_DESC_SHARE}/meshes/pgc_50_35/
        DESTINATION share/${PROJECT_NAME}/grippers/pgc_50_35/meshes)
install(DIRECTORY grippers config
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${GEN_DIR}/
        DESTINATION share/${PROJECT_NAME}
        PATTERN ".generated" EXCLUDE)

ament_package()
