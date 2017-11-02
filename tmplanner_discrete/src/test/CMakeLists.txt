catkin_add_gtest(test_map_update test_map_update.cpp)

include_directories(include)

target_link_libraries(test_map_update ${catkin_LIBRARIES} grid_map)

SET_TARGET_PROPERTIES(test_map_update PROPERTIES COMPILE_FLAGS "-std=c++11")