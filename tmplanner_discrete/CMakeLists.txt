cmake_minimum_required(VERSION 2.8.3)
project(tmplanner_discrete)

add_definitions(-std=c++11)
#add_definitions(-DEIGEN_DONT_VECTORIZE=1)
#add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(tmplanner_node src/node/tmplanner_node.cpp)
cs_add_library(grid_map src/grid_map.cpp)
cs_add_library(lattice src/lattice.cpp)
cs_add_library(tmplanner src/tmplanner.cpp)

add_subdirectory(src/test)

target_link_libraries(tmplanner_node ${catkin_LIBRARIES} tmplanner grid_map lattice)

cs_install()
cs_export()
