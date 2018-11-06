cmake_minimum_required(VERSION 2.8.3)
project(pointcloud2_clustering)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pointcloud_msgs
  pcl_conversions
  sensor_msgs
  std_msgs
  pcl_ros
  roscpp
)

find_package(PCL 1.8.1 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointcloud2_clustering src/clustering.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud2_clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
