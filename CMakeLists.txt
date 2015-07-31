cmake_minimum_required(VERSION 2.8.3)
project(marker_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
  cv_bridge
  	
)

set(Look3D_DIR /usr/local/look3d/cmake)
find_package(LooK3D)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)
link_directories(${EIGEN_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pub src/pub.cpp)
target_link_libraries(pub ${catkin_LIBRARIES})

add_executable(marker_tracker_ros  src/marker_tracker_ros.cpp)
target_link_libraries(marker_tracker_ros  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(plane_tracker  src/plane_tracker.cpp)
target_link_libraries(plane_tracker  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(dot_detection  src/dot_detection.cpp)
target_link_libraries(dot_detection  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(4dots_detection  src/4dots_detection.cpp)
target_link_libraries(4dots_detection  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(4dots_detection_clicking src/4dots_detection_clicking.cpp)
target_link_libraries(4dots_detection_clicking  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(dot_detection_from_topic  src/dot_detection_from_topic.cpp)
target_link_libraries(dot_detection_from_topic  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

add_executable(image_pub src/image_pub.cpp)
target_link_libraries(image_pub  ${catkin_LIBRARIES} ${Look3D_LIBRARIES} ${Look3D_EXTERNAL_LIBS} )

