cmake_minimum_required(VERSION 3.0.2)
project(wall_follow)

find_package(catkin REQUIRED COMPONENTS
      ackermann_msgs
      roscpp
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
        ackermann_msgs
        roscpp 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(FILES
launch/wall_follow.launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

file(GLOB SOURCES "src/*.cpp")

add_executable(wall_follow_node src/wall_follow_node.cpp)

target_link_libraries(wall_follow_node ${catkin_LIBRARIES})