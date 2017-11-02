catkin_add_gtest(test_cov_fun test_cov_fun.cpp)
catkin_add_gtest(test_polyfit test_polyfit.cpp)
catkin_add_gtest(test_lattice test_lattice.cpp)
catkin_add_gtest(test_grid_map test_grid_map.cpp)
catkin_add_gtest(test_map_update test_map_update.cpp)
catkin_add_gtest(test_cmaes test_cmaes.cpp)

include_directories(include)

target_link_libraries(test_cov_fun ${catkin_LIBRARIES})
target_link_libraries(test_polyfit ${catkin_LIBRARIES})
target_link_libraries(test_lattice ${catkin_LIBRARIES} grid_map lattice)
target_link_libraries(test_grid_map ${catkin_LIBRARIES} grid_map)
target_link_libraries(test_map_update ${catkin_LIBRARIES} grid_map)
target_link_libraries(test_cmaes ${catkin_LIBRARIES})


SET_TARGET_PROPERTIES(test_cov_fun PROPERTIES COMPILE_FLAGS "-std=c++11")
SET_TARGET_PROPERTIES(test_polyfit PROPERTIES COMPILE_FLAGS "-std=c++11")
SET_TARGET_PROPERTIES(test_lattice PROPERTIES COMPILE_FLAGS "-std=c++11")
SET_TARGET_PROPERTIES(test_grid_map PROPERTIES COMPILE_FLAGS "-std=c++11")
SET_TARGET_PROPERTIES(test_map_update PROPERTIES COMPILE_FLAGS "-std=c++11")
SET_TARGET_PROPERTIES(test_cmaes PROPERTIES COMPILE_FLAGS "-std=c++11")