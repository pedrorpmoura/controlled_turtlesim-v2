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


# add_executable(binary_multiplexer src/binary_multiplexer.cpp)
# target_link_libraries(binary_multiplexer ${catkin_LIBRARIES})

add_executable(binary_multiplexer_twist src/binary_multiplexer_twist.cpp src/binary_multiplexer.cpp)
target_link_libraries(binary_multiplexer_twist ${catkin_LIBRARIES})

add_executable(random_mover src/random_mover.cpp)
target_link_libraries(random_mover ${catkin_LIBRARIES})

add_executable(safety_controller src/safety_controller.cpp)
target_link_libraries(safety_controller ${catkin_LIBRARIES})

