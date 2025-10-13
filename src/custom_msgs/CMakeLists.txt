CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(custom_msgs)
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
  ADD_COMPILE_OPTIONS(
    -Wall
    -Wextra
    -Wpedantic)
ENDIF()

# find dependencies
FIND_PACKAGE(ament_cmake REQUIRED)
FIND_PACKAGE(std_msgs REQUIRED)
FIND_PACKAGE(trajectory_msgs REQUIRED)
FIND_PACKAGE(rosidl_default_generators REQUIRED)

SET(DEPS
    std_msgs
    trajectory_msgs
    geometry_msgs)

# Add your custom action/service/msg files here
ROSIDL_GENERATE_INTERFACES(
  ${PROJECT_NAME}
  "msg/LocatedObject.msg"
  "srv/MachineControl.srv"
  "action/ExecuteTrajectory.action"
  "action/Picture.action"
  DEPENDENCIES
  ${DEPS})

AMENT_PACKAGE()
