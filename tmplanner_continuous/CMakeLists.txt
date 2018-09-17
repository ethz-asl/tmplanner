cmake_minimum_required(VERSION 2.8.3)
project(tmplanner_continuous)

add_definitions(-std=c++11)
#add_definitions(-DEIGEN_DONT_VECTORIZE=1)
#add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(${PROJECT_NAME}_tmplanner_node src/node/tmplanner_node.cpp)
cs_add_executable(${PROJECT_NAME}_mapping_node src/node/mapping_node.cpp)
cs_add_library(${PROJECT_NAME}_grid_map src/grid_map.cpp)
cs_add_library(${PROJECT_NAME}_lattice src/lattice.cpp)
cs_add_library(${PROJECT_NAME}_tmplanner src/tmplanner.cpp)

add_subdirectory(src/test)

target_link_libraries(${PROJECT_NAME}_tmplanner_node ${catkin_LIBRARIES} ${PROJECT_NAME}_tmplanner ${PROJECT_NAME}_grid_map ${PROJECT_NAME}_lattice)
target_link_libraries(${PROJECT_NAME}_mapping_node ${catkin_LIBRARIES} ${PROJECT_NAME}_tmplanner ${PROJECT_NAME}_grid_map ${PROJECT_NAME}_lattice)

cs_install()
cs_export()