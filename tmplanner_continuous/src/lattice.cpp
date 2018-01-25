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

#include "tmplanner_continuous/lattice.h"
#include "tmplanner_tools/matlab_tools.h"

void Lattice::createLattice(const double& maximum_height,
                            const double& minimum_height,
                            const double& sensor_fov_angle_x,
                            const double& sensor_fov_angle_y,
                            const int& min_height_points,
                            const double& height_increment,
                            const grid_map::GridMap& grid_map) {
  CHECK_GT(grid_map.getData().size(), 0) << "Invalid grid map!";
  lattice_points_.clear();

  // Fit a line to determine variation in points vs. height.
  int result;
  double x_data[2] = {minimum_height, maximum_height};
  double y_data[2] = {(double)min_height_points, 1.0};
  double coefficients[2];
  result = polyfit(x_data, y_data, 2, 1, coefficients);

  for (double height = minimum_height; height <= maximum_height;
       height += height_increment) {
    int number_of_points = coefficients[1] * height + coefficients[0];
    std::vector<double> x_points;
    std::vector<double> y_points;

    // Only one point in the center.
    if (number_of_points == 1) {
      x_points = {grid_map.getPosition()(0) +
                  grid_map.getLength()(0) * grid_map.getResolution()(0) / 2.0};
      y_points = {grid_map.getPosition()(1) +
                  grid_map.getLength()(1) * grid_map.getResolution()(1) / 2.0};
    } else {
      auto half_image_edge_size =
          grid_map.getImageEdgeSize(height, sensor_fov_angle_x,
                                    sensor_fov_angle_y) /
          2.0;

      // Create the x-y grid on this height level.
      x_points = linspace(
          (double)(half_image_edge_size(0)) + grid_map.getPosition()(0),
          (double)(grid_map.getLength()(0) * grid_map.getResolution()(0) -
                   half_image_edge_size(0) + grid_map.getPosition()(0)),
          sqrt((double)(number_of_points)));
      y_points = linspace(
          (double)(half_image_edge_size(1)) + grid_map.getPosition()(1),
          (double)(grid_map.getLength()(1) * grid_map.getResolution()(1) -
                   half_image_edge_size(1) + grid_map.getPosition()(1)),
          sqrt((double)(number_of_points)));
    }

    for (auto i = x_points.begin(); i != x_points.end(); ++i) {
      for (auto j = y_points.begin(); j != y_points.end(); ++j) {
        geometry_msgs::Point lattice_point;
        lattice_point.x = *i;
        lattice_point.y = *j;
        lattice_point.z = height;
        lattice_points_.push_back(lattice_point);
      }
    }
  }
}