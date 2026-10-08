cmake_minimum_required(VERSION 3.8)
project(cell_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(dobot_description REQUIRED)

get_filename_component(DOBOT_SHARE "${dobot_description_DIR}/.." ABSOLUTE)
set(CR5_CONFIG "${DOBOT_SHARE}/config/cr5.yaml")
if(NOT EXISTS "${CR5_CONFIG}")
  message(FATAL_ERROR "dobot_description config not found: ${CR5_CONFIG}")
endif()

execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c "import yaml"
  RESULT_VARIABLE YAML_CHECK
  OUTPUT_QUIET ERROR_QUIET)
if(NOT YAML_CHECK EQUAL 0)
  message(FATAL_ERROR
    "PyYAML missing for ${Python3_EXECUTABLE} (sudo apt install python3-yaml)")
endif()

set(GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_config.py")
set(TEMPLATE_FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/templates/cr5_joint_limits.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/templates/kinematics.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/templates/moveit_controllers.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/templates/ompl_planning.yaml"
  "${CMAKE_CURRENT_SOURCE_DIR}/templates/pilz_cartesian_limits.yaml"
)

set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

# Keep in sync with EXPECTED_OUTPUTS in scripts/generate_config.py
set(GENERATED_FILES
  "${GEN_DIR}/config/cell.srdf"
  "${GEN_DIR}/config/cell.yaml"
  "${GEN_DIR}/config/cell_controllers.yaml"
  "${GEN_DIR}/config/cell_joint_limits.yaml"
  "${GEN_DIR}/config/cell_kinematics.yaml"
  "${GEN_DIR}/config/cell_moveit_controllers.yaml"
  "${GEN_DIR}/config/cell_ompl_planning.yaml"
  "${GEN_DIR}/config/pilz_cartesian_limits.yaml"
  "${GEN_DIR}/urdf/cell.urdf.xacro"
  "${GEN_DIR}/urdf/cell_mujoco.urdf.xacro"
)

add_custom_command(
  OUTPUT ${GENERATED_FILES}
  COMMAND "${Python3_EXECUTABLE}" "${GENERATOR}"
  --templates "${CMAKE_CURRENT_SOURCE_DIR}/templates"
  --cr5-config "${CR5_CONFIG}"
  --out "${GEN_DIR}"
  DEPENDS "${GENERATOR}" ${TEMPLATE_FILES} "${CR5_CONFIG}"
  COMMENT "Generating cell MoveIt / ros2_control / xacro config"
  VERBATIM
)
add_custom_target(generate_cell_config ALL DEPENDS ${GENERATED_FILES})
install(FILES config/cell_moveit.rviz
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY "${GEN_DIR}/config/"
  DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY "${GEN_DIR}/urdf/"
  DESTINATION share/${PROJECT_NAME}/urdf)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
