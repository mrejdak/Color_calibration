cmake_minimum_required(VERSION 3.0.2)
project(color_calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(color_calibration_node
  src/color_calibration.cpp
  src/calibration_node.cpp
)

add_executable(color_correction_node
  src/color_correction.cpp
  src/color_correction_node.cpp
)

target_link_libraries(
  color_calibration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(
  color_correction_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS color_calibration_node color_correction_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
