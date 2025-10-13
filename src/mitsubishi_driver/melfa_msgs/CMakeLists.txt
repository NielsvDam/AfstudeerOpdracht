cmake_minimum_required(VERSION 3.8)
project(melfa_msgs)
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GpioConfigure.srv"
  "srv/ModeConfigure.srv"
  "msg/GpioState.msg"
  "msg/GpioCommand.msg"
  "msg/ControllerType.msg"
  "msg/ControlMode.msg"
 )

ament_package()
