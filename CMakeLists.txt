cmake_minimum_required(VERSION 2.8.3)
project(ardu_offb)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mavros_msgs
  zedyolo_msgs
)

find_package(ZED 2 QUIET)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mav_offboard
#  CATKIN_DEPENDS geometry_msgs mavros_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OPENCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(/home/ahmed/darknet/build-release)
link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(ardu_offb_node src/ardu_offb_node.cpp)

target_link_libraries(ardu_offb_node
   ${catkin_LIBRARIES} )

add_executable(ardu_offb_thrust src/ardu_offb_thrust.cpp)
add_library(pid src/pid.cpp)

target_link_libraries(ardu_offb_thrust
   ${catkin_LIBRARIES} pid)

add_executable( bodytf src/static_tf_body.cpp)
target_link_libraries(bodytf  ${catkin_LIBRARIES} )

add_executable(zed_node src/zed_node.cpp)
target_link_libraries(zed_node 
                        ${catkin_LIBRARIES}
                        ${ZED_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        libdark.so
                        ${CUDA_LIBRARIES})

add_executable(detect_node src/detect_node.cpp)

target_link_libraries(detect_node 
                        ${catkin_LIBRARIES}
                        ${ZED_LIBRARIES}
                        ${OpenCV_LIBRARIES}
                        libdark.so
                        ${CUDA_LIBRARIES})