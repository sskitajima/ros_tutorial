cmake_minimum_required(VERSION 3.0.2)
project(ros_tutorial)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES temp
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ros_tutorial.cpp
# )

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(ros_tutorial_image_publisher src/image_publisher.cpp)
add_executable(ros_tutorial_image_subscriber src/image_subscriber.cpp)

# add_executable(ros_tutorial_get_parameter src/get_parameter.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ros_tutorial_image_publisher
  ${catkin_LIBRARIES}
)

target_link_libraries(ros_tutorial_image_subscriber
  ${catkin_LIBRARIES}
)

# target_link_libraries(ros_tutorial_get_parameter
#   ${catkin_LIBRARIES}
# )