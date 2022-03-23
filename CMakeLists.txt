cmake_minimum_required(VERSION 2.8.3)
project(phidgets_ros)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  rospy
  sensor_msgs
  cmake_modules
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(load_cell_node src/load_cell_reader.cpp)
#target_link_libraries(load_cell_node ${catkin_LIBRARIES} -lphidgets)


