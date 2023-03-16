cmake_minimum_required(VERSION 3.0.2)
project(mux_seg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_conversions
  std_msgs
  message_generation
)

add_message_files(
  FILES
  gpParam.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
	sensor_msgs
	message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)

#SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "-std=c++17 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
#SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

#Ground Plane Fitter
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(mux_seg src/mux_seg.cpp)
add_executable(mux_seg_range_only src/mux_seg_range_only.cpp)

target_include_directories(mux_seg PRIVATE
	${PCL_INCLUDE_DIRS}
)

target_include_directories(mux_seg_range_only PRIVATE
	${PCL_INCLUDE_DIRS}
)

target_link_libraries(mux_seg
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

target_link_libraries(mux_seg_range_only
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)
