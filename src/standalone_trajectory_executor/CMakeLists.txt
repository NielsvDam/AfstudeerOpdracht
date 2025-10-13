CMAKE_MINIMUM_REQUIRED(VERSION 3.8)
PROJECT(standalone_trajectory_executor)
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
FIND_PACKAGE(rclcpp_action REQUIRED)
FIND_PACKAGE(ament_index_cpp REQUIRED)
FIND_PACKAGE(custom_msgs REQUIRED)
FIND_PACKAGE(sensor_msgs REQUIRED)

SET(DEPENDENCIES
    rclcpp
    rclcpp_action
    ament_index_cpp
    custom_msgs
    sensor_msgs)

INCLUDE_DIRECTORIES(include)

SET(EXECUTABLE_NAME
    ${PROJECT_NAME})

SET(SOURCE_FILES
    src/TrajectoryExecutorNode.cpp
    src/JointVectorUtils.cpp)

ADD_EXECUTABLE(
  ${EXECUTABLE_NAME}
  src/main.cpp ${SOURCE_FILES})

TARGET_COMPILE_FEATURES(
  ${EXECUTABLE_NAME}
  PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

AMENT_TARGET_DEPENDENCIES(${EXECUTABLE_NAME} ${DEPENDENCIES})

INSTALL(
  TARGETS ${EXECUTABLE_NAME}
  DESTINATION lib/${PROJECT_NAME})

INSTALL(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# --------------------------------------------------
# --- the rest of the file is only about testing ---
# --------------------------------------------------

FIND_PACKAGE(ament_cmake_gmock REQUIRED)
FIND_PACKAGE(accessor REQUIRED)

SET(TEST_DEPENDENCIES
    ${DEPENDENCIES} accessor)

SET(TEST_FILES
    ${SOURCE_FILES}
    test/JointVectorUtils_Test.cpp)

AMENT_ADD_GMOCK(
  ${EXECUTABLE_NAME}_test
  test/main.cpp
  ${TEST_FILES}
  ${SOURCE_FILES})

TARGET_COMPILE_FEATURES(
  ${EXECUTABLE_NAME}_test
  PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

TARGET_INCLUDE_DIRECTORIES(
  ${EXECUTABLE_NAME}_test
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

AMENT_TARGET_DEPENDENCIES(${EXECUTABLE_NAME}_test ${TEST_DEPENDENCIES})

INSTALL(
  TARGETS ${EXECUTABLE_NAME}_test
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

AMENT_PACKAGE()
