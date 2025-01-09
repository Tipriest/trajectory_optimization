cmake_minimum_required(VERSION 2.8.12)
project(trajectory_optimization)

set(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
message("!!!CMAKE_CXX_FLAGS is " ${CMAKE_CXX_FLAGS_DEBUG})
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(ENV{MOSEKLM_LICENSE_FILE} "~/.mosek/mosek.lic")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    quadrotor_msgs
    grid_map_core
    grid_map_ros
    grid_map_rviz_plugin
    grid_map_msgs
    tf
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp grid_map_core grid_map_ros grid_map_msgs geometry_msgs
)

link_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/include)
include_directories(
    include
    SYSTEM
    third_party
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    # ${DECOMP_UTIL_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)
message("!!!catkin_INCLUDE_DIRS is  ${catkin_INCLUDE_DIRS}")

# add_compile_options(-mno-avx)

add_executable(trajectory_optimization
    src/main.cpp
    src/traj_optimize3d.cpp
    src/bezier_generator3d.cpp
    src/bezier_base.cpp
    src/grid_map.cpp
    src/traj_search3d.cpp
)
target_link_libraries(trajectory_optimization ${catkin_LIBRARIES} ${PCL_LIBRARIES} mosek64)

add_executable(publish_point 
    rviz_plugins/sub_publishpoint.cpp)

target_link_libraries(publish_point ${catkin_LIBRARIES})

# add_executable()