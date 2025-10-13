cmake_minimum_required(VERSION 3.8)
project(melfa_driver)
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_CXX_EXTENSIONS OFF)

IF(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang")
  ADD_COMPILE_OPTIONS(
    -w # Disable all warnings for third party code
  )
ENDIF()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# generate_parameter_library(
#   custom_io_params # cmake target name for the parameter library
#   src/custom_io_params.yaml # path to input yaml file
# )


include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/hardware_interface.cpp
  src/melfa_rt_exc.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
# target_link_libraries(${PROJECT_NAME}
#   custom_io_params
# )
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
pluginlib_export_plugin_description_file(hardware_interface melfa_driver.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
