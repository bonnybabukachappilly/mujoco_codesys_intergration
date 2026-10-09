cmake_minimum_required(VERSION 3.8)
project(assembly_station)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(dobot_description REQUIRED)
find_package(eoat_description REQUIRED)
find_package(mujoco_system REQUIRED)

get_filename_component(DOBOT_SHARE "${dobot_description_DIR}/.." ABSOLUTE)
get_filename_component(EOAT_SHARE "${eoat_description_DIR}/.." ABSOLUTE)
get_filename_component(MUJOCO_SHARE "${mujoco_system_DIR}/.." ABSOLUTE)

set(GENERATOR "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_station.py")
set(STATION_YAML "${CMAKE_CURRENT_SOURCE_DIR}/config/station_configuration.yaml")

set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")

set(GENERATED_FILES
  "${GEN_DIR}/config/station.srdf"
  "${GEN_DIR}/config/cell_moveit_controllers.yaml"
)

add_custom_command(
  OUTPUT ${GENERATED_FILES}
  COMMAND "${Python3_EXECUTABLE}" "${GENERATOR}"
  --station "${STATION_YAML}"
  --dobot-share "${DOBOT_SHARE}"
  --eoat-share "${EOAT_SHARE}"
  --mujoco-share "${MUJOCO_SHARE}"
  --out "${GEN_DIR}"
  DEPENDS
  "${GENERATOR}"
  "${STATION_YAML}"
  "${DOBOT_SHARE}/config/cr5.yaml"
  COMMENT "Generating assembly_station files"
  VERBATIM
  USES_TERMINAL
)
add_custom_target(generate_station ALL DEPENDS ${GENERATED_FILES})

install(
  DIRECTORY "${GEN_DIR}/config/"
  DESTINATION share/${PROJECT_NAME}/config
)

# install(
#   DIRECTORY "${GEN_DIR}/urdf/"
#   DESTINATION share/${PROJECT_NAME}/urdf
# )

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
