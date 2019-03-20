cmake_minimum_required(VERSION 2.8.3)
project(power_ina219)

add_compile_options(-std=c++11 -li2c)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs)

catkin_package(INCLUDE_DIRS include)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ina219_node src/ina219_activity.cpp src/ina219_node.cpp)

include_directories(${catkin_INCLUDE_DIRS})

target_link_libraries(ina219_node ${catkin_LIBRARIES})

install(TARGETS ina219_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

