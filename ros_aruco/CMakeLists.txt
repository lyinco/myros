cmake_minimum_required(VERSION 3.0.2)
project(aruco_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_eigen 
  tf2_geometry_msgs
  camera_models  
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package()

# 核心：把算法编成库
add_library(aruco_tracker_lib
  src/aruco_tracker.cpp
  src/utility/aruco_board.cpp
  src/utility/parameters.cpp
  src/utility/cam_calib.cpp
  src/utility/parameters.cpp
)
target_link_libraries(aruco_tracker_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
)

add_library(aruco_pub_lib
  src/arucoRosPub.cpp
  src/aruco_tracker.cpp
  src/utility/aruco_board.cpp
  src/utility/parameters.cpp
)
target_link_libraries(aruco_pub_lib
  aruco_tracker_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(aruco_node src/node_main.cpp)
target_link_libraries(aruco_node 
  aruco_tracker_lib
  aruco_pub_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# 节点链接这个库
# add_executable(aruco_node src/node_main.cpp)
#target_link_libraries(aruco_node
#  aruco_tracker_lib
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)