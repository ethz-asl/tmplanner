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

#ifndef LATTICE_H_
#define LATTICE_H_

#include <geometry_msgs/Point.h>

#include "tmplanner_continuous/grid_map.h"

class Lattice {
 public:
  Lattice() = default;
  Lattice(const Lattice&) = default;
  Lattice& operator=(const Lattice&) = default;
  ~Lattice() = default;

  const std::vector<geometry_msgs::Point>& getLatticePoints() const {
    return lattice_points_;
  }

  // Creates discretized pyramidal lattice grid for selecting points over a
  // 2-D environment grid map.
  void createLattice(const double& maximum_height, const double& minimum_height,
                     const double& sensor_fov_angle_x,
                     const double& sensor_fov_angle_y,
                     const int& min_height_points,
                     const double& height_increment,
                     const grid_map::GridMap& grid_map);

 private:
  // Multi-resolution pyramid lattice for 3-D grid search.
  std::vector<geometry_msgs::Point> lattice_points_;
};

#endif  // LATTICE_H_