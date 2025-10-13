CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(hmi)
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
FIND_PACKAGE(rclcpp REQUIRED)
FIND_PACKAGE(tf2_ros REQUIRED)
FIND_PACKAGE(geometry_msgs REQUIRED)
FIND_PACKAGE(Qt5Widgets REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)
FIND_PACKAGE(control_msgs REQUIRED)
FIND_PACKAGE(custom_msgs REQUIRED)

INCLUDE_DIRECTORIES(include)

SET(EXECUTABLE_NAME
    ${PROJECT_NAME})

SET(SOURCE_FILES
    src/main.cpp
    src/config/Config.cpp
    src/config/PackageSection.cpp
    src/config/ParameterSection.cpp
    src/ConfigurationNode.cpp
    src/HumanMachineInterface.cpp)

ADD_EXECUTABLE(
  ${EXECUTABLE_NAME}
  ${SOURCE_FILES})

TARGET_COMPILE_FEATURES(
  ${EXECUTABLE_NAME}
  PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

SET(DEPENDENCIES
    rclcpp
    tf2_ros
    geometry_msgs
    Qt5Widgets
    yaml-cpp
    control_msgs
    custom_msgs)

AMENT_TARGET_DEPENDENCIES(${EXECUTABLE_NAME} ${DEPENDENCIES})
TARGET_LINK_LIBRARIES(${EXECUTABLE_NAME} yaml-cpp)

INSTALL(
  TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME})

INSTALL(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

AMENT_PACKAGE()
