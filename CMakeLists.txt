cmake_minimum_required(VERSION 2.8.3)
project(mpc_atypical)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++11)

#ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  octomap_ros
  octomap_msgs
  voxblox_ros
)



include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

set(qpOASES_SRC /home/lyw/lib/qpOASES-3.2.1)
file(GLOB_RECURSE qpOASES_LIBS ${qpOASES_SRC}/src/*.cpp)


#ECBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

#EIGEN
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})


#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)




catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp roslib
   LIBRARIES optim_traj_gen
)


#BUILD

# module1 : standalone node for test 

#BUILD
add_executable(chomp_test_app src/atypical_main.cpp
  src/chomp_base.cpp
  src/chomp_utils.cpp
  src/chomp_subroutine.cpp
  src/chomp_ros_wrapper.cpp
  ${qpOASES_LIBS}
)
target_link_libraries(chomp_test_app
  ${catkin_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
  ${OCTOMAP_INCLUDE_DIRS}
)




