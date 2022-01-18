cmake_minimum_required(VERSION 3.0.2)
project(detect)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
mv_driver
cv_bridge
image_transport
sensor_msgs

)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES detect
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
find_package(OpenCV 4 REQUIRED)
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
src/detect.cpp
)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)

