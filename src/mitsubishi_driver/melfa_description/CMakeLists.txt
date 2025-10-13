cmake_minimum_required(VERSION 3.8)
project(melfa_description)
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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

install(
  DIRECTORY urdf launch config ros2_control rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
