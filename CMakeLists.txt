cmake_minimum_required(VERSION 2.8.3)
project(scan_3d)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  tf
  laser_geometry
)

find_package(PCL)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

add_executable(scan_3d 
  src/main.cpp
  src/scan_3d.h
  src/scan_3d.cpp
)
  
target_link_libraries(scan_3d 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(view_3d
  src/pcl_view.cpp
)

target_link_libraries(view_3d
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS scan_3d view_3d
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
