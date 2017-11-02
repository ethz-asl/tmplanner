cmake_minimum_required(VERSION 2.8.3)
project(tmplanner_tools)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)

cs_install()
cs_export()
