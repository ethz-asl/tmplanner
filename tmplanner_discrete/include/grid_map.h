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

#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <math.h>
#include <stdlib.h>
#include <algorithm>
#include <chrono>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <glog/logging.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"
#include "minkindr_conversions/kindr_xml.h"

#include "logger.h"

namespace grid_map {

// Observation submap - sensor footprint.
// upper_left   -------   upper_right
//             |       |
//             |       |
// lower_left   -------   lower_right
//
struct SubmapCoordinates {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d lower_left;
  Eigen::Vector2d upper_right;
  Eigen::Vector2d upper_left;
  Eigen::Vector2d lower_right;
};

class GridMap {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  GridMap() = default;
  GridMap(const GridMap&) = default;
  GridMap& operator=(const GridMap&) = default;
  ~GridMap() = default;

  // Sets the geometry of the 2D grid map, given:
  // environment dimensions [m], map resolution [m/cell], map origin [m],
  // occupancy thresholds (0.0-1.0).
  void setMapGeometry(const double& width, const double& height,
                      const double& resolution_x, const double& resolution_y,
                      const double& position_x, const double& position_y,
                      const double& upper_threshold,
                      const double& lower_threshold,
                      const std::string& frame_id);

  // Sets unknown values (0.5) for entire grid map.
  void fillUnknown();
  // Sets empty occupancy for entire grid map.
  void fillFree();
  // Manually sets data for entire grid map.
  void fillMapData(const Eigen::MatrixXd& data);

  // Computes the entropy of the occupancy grid map.
  double computeEntropy() const;
  // Computes the number of unclassified points on the occupancy grid map based
  // on occupancy thresholds.
  int computeNumberOfUnclassifiedPoints() const;

  // Calculates edge size [m] of an image observed from a height [m].
  Eigen::Vector2d getImageEdgeSize(const double& camera_height,
                                   const double& sensor_fov_angle_x,
                                   const double& sensor_fov_angle_y) const;
  // Calculates the co-ordinates of the observed corners of a submap,
  // assuming camera is perfectly aligned with grid map.
  SubmapCoordinates getRectSubmapCoordinates(
      const geometry_msgs::Point& camera_position,
      const Eigen::Vector2d& image_edge_size) const;

  // Updates the environment grid map based on a list of poses corresponding to
  // occupancy detections.
  void updateMapFromPoseArray(
      const geometry_msgs::PoseArray& detections_poses,
      const geometry_msgs::Pose& mav_pose,
      const kindr::minimal::QuatTransformation& T_IMU_CAM,
      const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
      const double& saturation_height, const cv::Mat& intrinsics_matrix,
      const std::vector<double>& true_positive_coeffs,
      const std::vector<double>& false_negative_coeffs);

  // Updates the occupancy grid map uncertainty based on a prediction of what
  // will be seen from a location given the camera parameters.
  void predictMapUpdate(const geometry_msgs::Pose& mav_pose,
                        const kindr::minimal::QuatTransformation& T_IMU_CAM,
                        const double& sensor_fov_angle_x,
                        const double& sensor_fov_angle_y,
                        const double& saturation_height,
                        const std::vector<double>& true_positive_coeffs,
                        const std::vector<double>& false_negative_coeffs);

  // Projects a pixel in a camera image to the ground plane based on
  // roll-pitch-yaw angles.
  Eigen::Vector4d projectPixelToGround(
      const cv::Mat& intrinsics_matrix,
      const kindr::minimal::QuatTransformation& T_MAP_CAM,
      const Eigen::Vector4d& point);

  Eigen::MatrixXd getData() const { return data_; }
  std::string getFrameId() const { return frame_id_; }
  Eigen::Vector2i getLength() const { return length_; }
  Eigen::Vector2d getPosition() const { return position_; }
  Eigen::Vector2d getResolution() const { return resolution_; }

  // Converts the 2D occupancy map into a ROS occupancy grid message.
  void toOccupancyGrid(nav_msgs::OccupancyGrid* occupancy_grid);

 private:
  // Occupancy grid map data (log-odds).
  Eigen::MatrixXd data_;

  // Frame ID of the map.
  std::string frame_id_;
  // Side lengths of the map [cells].
  Eigen::Vector2i length_;
  // Position of origin in the grid map frame [m].
  Eigen::Vector2d position_;
  // Map resolution in each dimension [m/cell].
  Eigen::Vector2d resolution_;
  // Thresholds for map occupancy values (0.0-1.0).
  double upper_threshold_;
  double lower_threshold_;

  MappingLogger logger_;

  // Calculates the probability updates for an altitude-dependent sensor.
  double computeTruePositiveProbability(
      const double& camera_height, const double& saturation_height,
      const std::vector<double>& true_positive_coeffs) const;
  double computeFalseNegativeProbability(
      const double& camera_height, const double& saturation_height,
      const std::vector<double>& false_negative_coeffs) const;

  // Converts co-ordinates from grid map to environment representation.
  void gridToEnvironmentCoordinates(geometry_msgs::Point* point) const;
  // Converts co-ordinates from environment to grid map representation.
  void environmentToGridCoordinates(geometry_msgs::Point* point) const;
  void environmentToGridCoordinates(Eigen::Vector2d* point) const;
  void environmentToGridCoordinates(
      SubmapCoordinates* submap_coordinates) const;
  // Crops submap co-ordinates to nearest valid bounds given the map size.
  void trimSubmapCoordinatesToGrid(SubmapCoordinates* submap_coordinates) const;

  // Converts the log odds of a single occupancy grid map cell to a probability
  // value (0-100).
  double toProbability(double x) const;
  // Converts the probability (0-1) of a single occupancy grid map cell to a log
  // odds value.
  double toLogOdds(double x) const;
};
}  // namespace grid_map

#endif  // GRID_MAP_H_