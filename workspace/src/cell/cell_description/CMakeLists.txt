cmake_minimum_required(VERSION 3.8)
project(cell_description)

find_package(ament_cmake REQUIRED)
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

# ---- generation (build time, dependency-tracked) ----
set(GEN_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_cell.py)
set(GEN_DIR    ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GEN_STAMP  ${GEN_DIR}/.generated)

add_custom_command(
  OUTPUT ${GEN_STAMP}
  COMMAND ${CMAKE_COMMAND} -E remove_directory ${GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${GEN_SCRIPT} urdf
          --cell-share ${CMAKE_CURRENT_SOURCE_DIR}
          --out ${GEN_DIR} --stamp ${GEN_STAMP}
  DEPENDS ${GEN_SCRIPT}
          ${CMAKE_CURRENT_SOURCE_DIR}/config/cell.yaml
          ${CMAKE_CURRENT_SOURCE_DIR}/config/grippers.yaml
  COMMENT "Generating cell URDF"
  VERBATIM)
add_custom_target(generate_cell_urdf ALL DEPENDS ${GEN_STAMP})

# ---- install ----
# scripts/ is installed so that cell_mujoco and cell_moveit_config can run the
# same generator against the same cell.yaml (single source of truth).
install(DIRECTORY config scripts
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${GEN_DIR}/
        DESTINATION share/${PROJECT_NAME}
        PATTERN ".generated" EXCLUDE)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(generator test TIMEOUT 120)
endif()

ament_package()
