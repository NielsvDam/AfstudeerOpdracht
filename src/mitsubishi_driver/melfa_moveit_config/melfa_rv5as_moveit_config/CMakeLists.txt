CMAKE_MINIMUM_REQUIRED(VERSION 3.22)
PROJECT(melfa_rv5as_moveit_config)
SET(CMAKE_EXPORT_COMPILE_COMMANDS
    ON)

SET(CMAKE_CXX_STANDARD
    17)
SET(CMAKE_CXX_STANDARD_REQUIRED
    ON)
SET(CMAKE_CXX_EXTENSIONS
    OFF)

IF(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang")
  ADD_COMPILE_OPTIONS(-w # Disable all warnings for third party code
  )
ENDIF()

FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(ament_cmake_python REQUIRED)
FIND_PACKAGE(control_msgs REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(rclcpp_action REQUIRED)
FIND_PACKAGE(tf2_geometry_msgs REQUIRED)

AMENT_PACKAGE()

INSTALL(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
INSTALL(
  DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME})
INSTALL(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME})

ADD_EXECUTABLE(
  servo_keyboard_input
  src/servo_keyboard_input.cpp)
TARGET_INCLUDE_DIRECTORIES(
  servo_keyboard_input
  PUBLIC include)
AMENT_TARGET_DEPENDENCIES(
  servo_keyboard_input
  ament_cmake
  rclcpp
  rclcpp_action
  tf2_geometry_msgs
  control_msgs)

INSTALL(
  TARGETS servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME})
