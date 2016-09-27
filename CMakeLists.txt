cmake_minimum_required(VERSION 2.8.3)
project(rov_controller)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  joy
)

catkin_package(

)




include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(joystick_mapper src/joystick_mapper.cpp)
target_link_libraries(joystick_mapper ${catkin_LIBRARIES})
add_dependencies(joystick_mapper beginner_tutorials_generate_messages_cpp)

add_executable(rov_controller_node src/rov_controller_node.cpp)
target_link_libraries(rov_controller_node ${catkin_LIBRARIES})
add_dependencies(rov_controller_node beginner_tutorials_generate_messages_cpp)
