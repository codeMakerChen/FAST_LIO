cmake_minimum_required(VERSION 2.8.3)
project(fast_lio)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

option(FASTLIO_BUILD_ROS "Build original ROS node" ON)
option(FASTLIO_BUILD_OFFLINE "Build ROS-independent offline executable" ON)

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  find_path(EIGEN3_INCLUDE_DIR Eigen/Core PATHS /usr/include/eigen3 /usr/local/include/eigen3)
  if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR "Eigen3 not found")
  endif()
else()
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()
find_package(PCL 1.8 REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  include)

if(FASTLIO_BUILD_ROS)
  find_package(PythonLibs REQUIRED)
  find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    sensor_msgs
    roscpp
    rospy
    std_msgs
    pcl_ros
    tf
    livox_ros_driver
    message_generation
    eigen_conversions
  )

  include_directories(${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

  add_message_files(
    FILES
    Pose6D.msg
  )

  generate_messages(
   DEPENDENCIES
   geometry_msgs
  )

  catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS
  )

  add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
  target_link_libraries(fastlio_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
  target_include_directories(fastlio_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})
endif()

if(FASTLIO_BUILD_OFFLINE)
  add_executable(fastlio_offline offline/offline_main.cpp)
  target_link_libraries(fastlio_offline ${PCL_LIBRARIES})
endif()
