cmake_minimum_required(VERSION 3.0.2)
project(controlled_turtlesim-v2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_package()


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



add_executable(binary_multiplexer_twist src/binary_multiplexer_twist.cpp)
target_link_libraries(binary_multiplexer_twist ${catkin_LIBRARIES})

add_executable(random_mover src/random_mover.cpp)
target_link_libraries(random_mover ${catkin_LIBRARIES})
