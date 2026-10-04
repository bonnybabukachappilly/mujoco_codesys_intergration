cmake_minimum_required(VERSION 3.8)
project(cell_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(cell_description REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import yaml"
  RESULT_VARIABLE yaml_rc OUTPUT_QUIET ERROR_QUIET)
if(NOT yaml_rc EQUAL 0)
  message(FATAL_ERROR
    "PyYAML missing for ${Python3_EXECUTABLE}. "
    "Install python3-yaml or pass -DPython3_EXECUTABLE=/usr/bin/python3")
endif()

get_filename_component(CELL_DESC_SHARE "${cell_description_DIR}/.." ABSOLUTE)

# ---- generation (build time, dependency-tracked) ----
set(GEN_SCRIPT ${CELL_DESC_SHARE}/scripts/generate_cell.py)
set(GEN_DIR    ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GEN_STAMP  ${GEN_DIR}/.generated)

add_custom_command(
  OUTPUT ${GEN_STAMP}
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${GEN_SCRIPT} moveit
          --cell-share ${CELL_DESC_SHARE}
          --out ${GEN_DIR} --stamp ${GEN_STAMP}
  DEPENDS ${GEN_SCRIPT}
          ${CELL_DESC_SHARE}/config/cell.yaml
          ${CELL_DESC_SHARE}/config/grippers.yaml
  COMMENT "Generating MoveIt configuration"
  VERBATIM)
add_custom_target(generate_cell_moveit ALL DEPENDS ${GEN_STAMP})

# ---- install ----
install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${GEN_DIR}/
        DESTINATION share/${PROJECT_NAME}
        PATTERN ".generated" EXCLUDE)

ament_package()
