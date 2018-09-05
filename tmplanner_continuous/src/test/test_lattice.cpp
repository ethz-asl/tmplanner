/*
 * Copyright (c) 2017, Marija Popovic, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "tmplanner_continuous/grid_map.h"
#include "tmplanner_continuous/lattice.h"

TEST(LatticeTest, CreatesLattice) {
  // Lattice parameters
  // const double maximum_height = 26.0;
  // const double minimum_height = 1.0;
  // const double sensor_fov_angle_x = 45.0;
  // const double sensor_fov_angle_y = 60.0;
  // const int min_height_points = 16;
  // const double height_increment = 8.0;
  // Raven - Bonn
  // const double maximum_height = 7.61;
  // const double minimum_height = 2.0;
  // const double sensor_fov_angle_x = 42.7;
  // const double sensor_fov_angle_y = 55.0;
  // const int min_height_points = 9;
  // const double height_increment = 2.8;
  // Raven - Zurich
  const double maximum_height = 21.1;
  const double minimum_height = 8.0;
  const double sensor_fov_angle_x = 42.7;
  const double sensor_fov_angle_y = 55.0;
  const int min_height_points = 25;
  const double height_increment = 6.5;

  // Grid map parameters
  // const double width = 30.0;
  // const double height = 30.0;
  // const double resolution_x = 0.75;
  // const double resolution_y = 0.75;
  // Raven - Bonn
  // const double width = 8.0;
  // const double height = 8.0;
  // const double resolution_x = 0.20;
  // const double resolution_y = 0.20;
  // Raven - Zurich
  const double width = 20.0;
  const double height = 20.0;
  const double resolution_x = 0.50;
  const double resolution_y = 0.50;

  // Create the grid map.
  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, "map");

  // Create the lattice.
  Lattice lattice;
  lattice.createLattice(maximum_height, minimum_height, sensor_fov_angle_x,
                        sensor_fov_angle_y, min_height_points, height_increment,
                        grid_map);
  std::vector<geometry_msgs::Point> points = lattice.getLatticePoints();
  //EXPECT_EQ(points.size(), 30);
  for (size_t i = 0; i < points.size(); ++i) {
     LOG(INFO) << points[i].x << ", " << points[i].y << ", " << points[i].z
               << ";";
  }
}

// gtest main
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}