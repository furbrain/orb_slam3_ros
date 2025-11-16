cmake_minimum_required(VERSION 3.5)
project(orb_slam3)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall -O3 -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -fPIC -Wno-reorder -Wno-deprecated-declarations -Wno-unused-but-set-variable -Wno-unused-parameter")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})
find_package(Eigen3 3.1.0 REQUIRED)


# ament stuff
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(ament_deps
  cv_bridge
  image_transport
  nav_msgs
  message_filters
  rclcpp
  tf2_ros
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  Boost
)

set(orb_targets
  #ros_mono
  #ros_mono_inertial
  #ros_stereo
  #ros_stereo_inertial
  #ros_rgbd
  ros_rgbd_inertial
)

set(srv_files
  srv/SaveMap.srv
)
 
set(msg_files
  msg/Atlas.msg
  msg/Map.msg
  msg/KeyFrame.msg
  msg/KeyPoint.msg
  msg/Point3D.msg
  msg/NumPoints.msg
  msg/State.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(Boost rclcpp rclpy std_msgs cv_bridge image_transport tf2 tf2_ros sensor_msgs rosidl_default_runtime)
ament_export_libraries({PROJECT_NAME} libDBoW2 libg2o)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
)

include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

add_library(${PROJECT_NAME}_lib SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
  orb_slam3/include/System.h
  orb_slam3/include/Tracking.h
  orb_slam3/include/LocalMapping.h
  orb_slam3/include/LoopClosing.h
  orb_slam3/include/ORBextractor.h
  orb_slam3/include/ORBmatcher.h
  orb_slam3/include/FrameDrawer.h
  orb_slam3/include/Converter.h
  orb_slam3/include/MapPoint.h
  orb_slam3/include/KeyFrame.h
  orb_slam3/include/Atlas.h
  orb_slam3/include/Map.h
  orb_slam3/include/Optimizer.h
  orb_slam3/include/Frame.h
  orb_slam3/include/KeyFrameDatabase.h
  orb_slam3/include/Sim3Solver.h
  orb_slam3/include/ImuTypes.h
  orb_slam3/include/G2oTypes.h
  orb_slam3/include/CameraModels/GeometricCamera.h
  orb_slam3/include/CameraModels/Pinhole.h
  orb_slam3/include/CameraModels/KannalaBrandt8.h
  orb_slam3/include/OptimizableTypes.h
  orb_slam3/include/MLPnPsolver.h
  orb_slam3/include/GeometricTools.h
  orb_slam3/include/TwoViewReconstruction.h
  orb_slam3/include/SerializationUtils.h
  orb_slam3/include/Config.h
  orb_slam3/include/Settings.h
)

target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lboost_filesystem
  -lcrypto
)

## ROS node

## ROS node
#add_executable(ros_mono_inertial 
#  src/ros_mono_inertial.cc
#  src/common.cc
#)
#target_link_libraries(ros_mono_inertial 
#  ${PROJECT_NAME}_lib
#)
#rosidl_target_interfaces(ros_mono_inertial
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")


## ROS node
#add_executable(ros_stereo 
#  src/ros_stereo.cc
#  src/common.cc
#)
#target_link_libraries(ros_stereo
#  ${PROJECT_NAME}_lib
#)
#rosidl_target_interfaces(ros_stereo
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")


## ROS node
#add_executable(ros_stereo_inertial
#  src/ros_stereo_inertial.cc
#  src/common.cc
#)
#target_link_libraries(ros_stereo_inertial 
#  ${PROJECT_NAME}_lib
#)
#rosidl_target_interfaces(ros_stereo_inertial
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")


## ROS node
#add_executable(ros_rgbd
#  src/ros_rgbd.cc
#  src/common.cc
#)
#target_link_libraries(ros_rgbd
#  ${PROJECT_NAME}_lib
#)
#rosidl_target_interfaces(ros_rgbd
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")


## ROS node
#add_executable(ros_rgbd_inertial
#  src/ros_rgbd_inertial.cc
#  src/common.cc
#)
#target_link_libraries(ros_rgbd_inertial
#  ${PROJECT_NAME}_lib
#)
#rosidl_target_interfaces(ros_rgbd_inertial
#  ${PROJECT_NAME} "rosidl_typesupport_cpp")

foreach(tgt IN LISTS orb_targets)
  add_executable(${tgt}
    src/${tgt}.cc
    src/common.cc
  )
  target_link_libraries(${tgt} ${PROJECT_NAME}_lib)
  rosidl_target_interfaces(${tgt} ${PROJECT_NAME} "rosidl_typesupport_cpp")
  ament_target_dependencies(${tgt} ${ament_deps})  
endforeach()


#ament_target_dependencies(ros_mono_inertial ${ament_deps})
#ament_target_dependencies(ros_stereo ${ament_deps})
#ament_target_dependencies(ros_stereo_inertial ${ament_deps})
#ament_target_dependencies(ros_rgbd ${ament_deps})
#ament_target_dependencies(ros_rgbd_inertial ${ament_deps})

install(TARGETS
  ${orb_targets}
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  orb_slam3/Vocabulary/ORBvoc.txt.bin
  DESTINATION share/${PROJECT_NAME}/vocab)

ament_package()
