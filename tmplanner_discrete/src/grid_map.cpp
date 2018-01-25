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

#include "tmplanner_discrete/grid_map.h"

using namespace grid_map;

void GridMap::setMapGeometry(const double& width, const double& height,
                             const double& resolution_x,
                             const double& resolution_y,
                             const double& position_x, const double& position_y,
                             const double& upper_threshold,
                             const double& lower_threshold,
                             const std::string& frame_id) {
  CHECK_GT(resolution_x, 0) << "Error in map resolution value in x.";
  CHECK_GT(resolution_y, 0) << "Error in map resolution value in y.";
  CHECK_GE(width, 0) << "Error in map width value.";
  CHECK_GE(height, 0) << "Error in map height value.";
  CHECK_LE(upper_threshold, 1.0) << "Error in map threshold values";
  CHECK_LE(lower_threshold, 1.0) << "Error in map threshold values";
  CHECK_GE(upper_threshold, 0.0) << "Error in map threshold values";
  CHECK_GE(lower_threshold, 0.0) << "Error in map threshold values";

  frame_id_ = frame_id;
  resolution_(0) = resolution_x;
  resolution_(1) = resolution_y;
  length_(0) = static_cast<int>(round(height / resolution_x));
  length_(1) = static_cast<int>(round(width / resolution_y));

  data_.resize(length_(0), length_(1));

  // Center the map corners.
  position_(0) = position_x;
  position_(1) = position_y;
}

void GridMap::fillUnknown() { data_.setZero(length_(0), length_(1)); }

void GridMap::fillFree() {
  data_.setOnes(length_(0), length_(1));
  data_ = data_ * -std::numeric_limits<double>::infinity();
}

void GridMap::fillMapData(const Eigen::MatrixXd& data) {
  CHECK_EQ(length_(0), data.rows()) << "Error in GridMap::fillMapData! Data "
                                       "dimensions don't match map dimensions.";
  CHECK_EQ(length_(1), data.cols()) << "Error in GridMap::fillMapData! Data "
                                       "dimensions don't match map dimensions.";
  data_ = data;
}

double GridMap::computeEntropy() const {
  double entropy = 0;
  for (auto i = 0; i < data_.rows(); ++i) {
    for (auto j = 0; j < data_.cols(); ++j) {
      double p = toProbability(data_(i, j));
      // Set unknown occupancy.
      if (p == -1.0 / 100) {
        p = 0.5;
      }
      CHECK_GT(p, 0) << "Invalid probability in map cell.";
      CHECK_LT(p, 1) << "Invalid probability in map cell.";
      entropy += -(p * std::log(p) + (1.0 - p) * std::log(1.0 - p));
    }
  }
  return entropy;
}

int GridMap::computeNumberOfUnclassifiedPoints() const {
  int number_of_unclassified_points = 0;
  for (auto i = 0; i < data_.rows(); ++i) {
    for (auto j = 0; j < data_.cols(); ++j) {
      double p = toProbability(data_(i, j));
      if ((p > lower_threshold_ && p < upper_threshold_) || p == -1.0 / 100)
        number_of_unclassified_points++;
    }
  }
  return number_of_unclassified_points;
}

Eigen::Vector2d GridMap::getImageEdgeSize(
    const double& camera_height, const double& sensor_fov_angle_x,
    const double& sensor_fov_angle_y) const {
  CHECK_GT(camera_height, 0.0) << "Camera height is lower than 0.0m.";
  Eigen::Vector2d image_edge_size;
  image_edge_size(0) =
      (2.0 * camera_height) * tan((sensor_fov_angle_x / 2.0) * (M_PI / 180.0));
  image_edge_size(1) =
      (2.0 * camera_height) * tan((sensor_fov_angle_y / 2.0) * (M_PI / 180.0));
  return image_edge_size;
}

SubmapCoordinates GridMap::getRectSubmapCoordinates(
    const geometry_msgs::Point& camera_position,
    const Eigen::Vector2d& image_edge_size) const {
  SubmapCoordinates submap_coordinates;
  submap_coordinates.lower_left(0) =
      camera_position.x - (image_edge_size(0) / 2.0);
  submap_coordinates.lower_left(1) =
      camera_position.y - (image_edge_size(1) / 2.0);
  submap_coordinates.upper_right(0) =
      camera_position.x + (image_edge_size(0) / 2.0);
  submap_coordinates.upper_right(1) =
      camera_position.y + (image_edge_size(1) / 2.0);
  submap_coordinates.upper_left(0) =
      camera_position.x - (image_edge_size(0) / 2.0);
  submap_coordinates.upper_left(1) =
      camera_position.y + (image_edge_size(1) / 2.0);
  submap_coordinates.lower_right(0) =
      camera_position.x + (image_edge_size(0) / 2.0);
  submap_coordinates.lower_right(1) =
      camera_position.y - (image_edge_size(1) / 2.0);
  return submap_coordinates;
}

void GridMap::updateMapFromPoseArray(
    const geometry_msgs::PoseArray& detections_poses,
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const double& saturation_height, const cv::Mat& intrinsics_matrix,
    const std::vector<double>& true_positive_coeffs,
    const std::vector<double>& false_negative_coeffs) {
  // Obtain world to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute map to camera transform.
  kindr::minimal::QuatTransformation T_MAP_CAM = T_MAP_IMU * T_IMU_CAM;
  geometry_msgs::Pose camera_pose;
  tf::poseKindrToMsg(T_MAP_CAM, &camera_pose);

  // Calculate the corner co-ordinates of the observed submap.
  // NB: - Assume that principal point is at the center of the image.
  SubmapCoordinates submap_coordinates;
  Eigen::Vector4d p1;
  p1 << 0.0, 0.0, 1.0, 1.0;
  Eigen::Vector4d p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.upper_left = p2.head(2);
  p1 << 0.0, (intrinsics_matrix.at<double>(1, 2) * 2.0 - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_left = p2.head(2);
  p1 << (intrinsics_matrix.at<double>(0, 2) * 2.0 - 1.0),
      (intrinsics_matrix.at<double>(1, 2) * 2.0 - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_right = p2.head(2);
  p1 << (intrinsics_matrix.at<double>(0, 2) * 2.0 - 1.0), 0.0, 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.upper_right = p2.head(2);
  environmentToGridCoordinates(&submap_coordinates);
  trimSubmapCoordinatesToGrid(&submap_coordinates);

  // Initialize variables to keep track of submap data.
  const size_t x_min = std::min(
      {submap_coordinates.lower_left(0), submap_coordinates.upper_left(0)});
  const size_t x_max = std::max(
      {submap_coordinates.upper_right(0), submap_coordinates.lower_right(0)});
  const size_t y_min = std::min(
      {submap_coordinates.lower_left(1), submap_coordinates.lower_right(1)});
  const size_t y_max = std::max(
      {submap_coordinates.upper_left(1), submap_coordinates.upper_right(1)});
  const size_t submap_size_x = x_max - x_min + 1;
  const size_t submap_size_y = y_max - y_min + 1;
  Eigen::MatrixXd submap;
  submap.setZero(submap_size_y, submap_size_x);

  // Project each tag onto submap, and set corresponding cell to "1".
  for (size_t i = 0; i < detections_poses.poses.size(); ++i) {
    // Get camera to tag transform for this tag.
    kindr::minimal::QuatTransformation T_CAM_TAG;
    tf::poseMsgToKindr(detections_poses.poses[i], &T_CAM_TAG);
    // Compute world to tag transform.
    kindr::minimal::QuatTransformation T_MAP_TAG = T_MAP_CAM * T_CAM_TAG;
    geometry_msgs::Pose tag_pose;
    tf::poseKindrToMsg(T_MAP_TAG, &tag_pose);
    // Determine occupied cell.
    geometry_msgs::Point tag_position_grid = tag_pose.position;
    environmentToGridCoordinates(&tag_position_grid);
    // Skip measurement: out-of-bounds.
    if (tag_position_grid.x - x_min < 0 ||
        tag_position_grid.x - x_min > (submap_size_x - 1) ||
        tag_position_grid.y - y_min < 0 ||
        tag_position_grid.y - y_min > (submap_size_y - 1) ||
        tag_position_grid.x < 0 || tag_position_grid.y < 0 ||
        tag_position_grid.x > (length_(0) - 1) ||
        tag_position_grid.y > (length_(1) - 1)) {
      continue;
    }
    LOG(INFO) << "Detection at x = " << tag_pose.position.x
              << ", y = " << tag_pose.position.y << ".";
    submap(tag_position_grid.y - y_min, tag_position_grid.x - x_min) = 1.0;
  }

  // Perform occupancy map update for each cell in the submap.
  for (size_t i = 0; i < submap.rows(); ++i) {
    for (size_t j = 0; j < submap.cols(); ++j) {
      // Occupancy probability given the current measurement.
      double p;
      // Occupied: true positive sensor model.
      if (submap(i, j) == 1) {
        p = computeTruePositiveProbability(
            camera_pose.position.z, saturation_height, true_positive_coeffs);
        // Free: false positive sensor model.
      } else {
        p = computeFalseNegativeProbability(
            camera_pose.position.z, saturation_height, false_negative_coeffs);
      }
      data_(y_min + i, x_min + j) += std::log(p / (1 - p));
    }
  }
}

void GridMap::predictMapUpdate(
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const double& saturation_height,
    const std::vector<double>& true_positive_coeffs,
    const std::vector<double>& false_negative_coeffs) {
  // Obtain map to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute map to camera transform.
  kindr::minimal::QuatTransformation T_MAP_CAM = T_MAP_IMU * T_IMU_CAM;
  geometry_msgs::Pose camera_pose;
  tf::poseKindrToMsg(T_MAP_CAM, &camera_pose);

  // Calculate the image coverage [m] based on the camera position,
  // assuming camera is perfectly aligned with the map.
  const auto image_edge_size = getImageEdgeSize(
      camera_pose.position.z, sensor_fov_angle_x, sensor_fov_angle_y);

  // Find bounds of the image given the current position.
  const auto submap_coordinates =
      getRectSubmapCoordinates(camera_pose.position, image_edge_size);
  auto submap_coordinates_grid = submap_coordinates;
  environmentToGridCoordinates(&submap_coordinates_grid);
  trimSubmapCoordinatesToGrid(&submap_coordinates_grid);

  // Update all points inside observed submap assuming ML measurements.
  for (size_t i = submap_coordinates_grid.lower_left(0);
       i <= submap_coordinates_grid.upper_right(0); ++i) {
    for (size_t j = submap_coordinates_grid.lower_left(1);
         j <= submap_coordinates_grid.upper_right(1); ++j) {
      // Occupancy probability given the current measurement.
      double p;
      // Occupied: True positive sensor model.
      if (data_(j, i) > 0) {
        p = computeTruePositiveProbability(
            camera_pose.position.z, saturation_height, true_positive_coeffs);
      }
      // Free: False positive sensor model.
      else {
        p = computeFalseNegativeProbability(
            camera_pose.position.z, saturation_height, false_negative_coeffs);
      }
      data_(j, i) += std::log(p / (1 - p));
    }
  }
}

Eigen::Vector4d GridMap::projectPixelToGround(
    const cv::Mat& intrinsics_matrix,
    const kindr::minimal::QuatTransformation& T_MAP_CAM,
    const Eigen::Vector4d& point) {
  // Full-rank camera extrinsics matrix for the homography.
  Eigen::Matrix4d E = T_MAP_CAM.getTransformationMatrix();
  // Full-rank camera calibration intrinsics matrix for the homography.
  Eigen::Matrix4d K;
  K << intrinsics_matrix.at<double>(0, 0), 0,
      intrinsics_matrix.at<double>(0, 2), 0, 0,
      intrinsics_matrix.at<double>(1, 1), intrinsics_matrix.at<double>(1, 2), 0,
      0, 0, 1, 0, 0, 0, 0, 1;
  // Full-rank camera matrix (intrinsics + extrinsics).
  Eigen::Matrix4d P = K * E.inverse();

  // Apply the homography transform to find the ray projected from the camera.
  Eigen::Vector4d camera_point = E.block(0, 3, 1, 4);
  Eigen::Vector4d ray_vector = (P.inverse() * point) - camera_point;
  // Find the intersection of the image ray and the ground plane.
  return camera_point - (camera_point(2) / ray_vector(2)) * ray_vector;
}

void GridMap::toOccupancyGrid(nav_msgs::OccupancyGrid* occupancy_grid) {
  CHECK_NOTNULL(occupancy_grid);
  occupancy_grid->header.frame_id = frame_id_;
  occupancy_grid->header.stamp = ros::Time::now();
  occupancy_grid->info.map_load_time = occupancy_grid->header.stamp;
  // Map dimensions [cells].
  occupancy_grid->info.width = length_(0);
  occupancy_grid->info.height = length_(1);
  // Assume same resolution in both grid dimensions.
  occupancy_grid->info.resolution = resolution_(0);
  // Map position [m].
  occupancy_grid->info.origin.position.x = position_(0);
  occupancy_grid->info.origin.position.y = position_(1);
  occupancy_grid->info.origin.position.z = 0.0;
  occupancy_grid->info.origin.orientation.x = 0.0;
  occupancy_grid->info.origin.orientation.y = 0.0;
  occupancy_grid->info.origin.orientation.z = 0.0;
  occupancy_grid->info.origin.orientation.w = 1.0;
  occupancy_grid->data.resize(length_(0) * length_(1));
  // Row-major ordering.
  for (auto i = 0; i < data_.rows(); ++i) {
    for (auto j = 0; j < data_.cols(); ++j) {
      occupancy_grid->data[(j + i * data_.cols())] =
          toProbability(data_(i, j)) * 100;
    }
  }
}

double GridMap::computeTruePositiveProbability(
    const double& camera_height, const double& saturation_height,
    const std::vector<double>& true_positive_coeffs) const {
  if (camera_height > saturation_height) {
    return 0.5;
  } else {
    return true_positive_coeffs[0] * pow(camera_height, 2) +
           true_positive_coeffs[1] * camera_height + true_positive_coeffs[2];
  }
}

double GridMap::computeFalseNegativeProbability(
    const double& camera_height, const double& saturation_height,
    const std::vector<double>& false_negative_coeffs) const {
  if (camera_height > saturation_height) {
    return 0.5;
  } else {
    return false_negative_coeffs[0] * pow(camera_height, 2) +
           false_negative_coeffs[1] * camera_height + false_negative_coeffs[2];
  }
}

void GridMap::gridToEnvironmentCoordinates(geometry_msgs::Point* point) const {
  // Add half a cell size in the conversion to get the center of the cell.
  point->x = point->x * resolution_(0) + position_(0) + resolution_(0) / 2.0;
  point->y = point->y * resolution_(1) + position_(1) + resolution_(1) / 2.0;
}

void GridMap::environmentToGridCoordinates(geometry_msgs::Point* point) const {
  // Round down here, because the points here are indices to the grid data
  // structure (starting from 0).
  double f1;
  std::modf((point->x - position_(0)) / resolution_(0), &f1);
  point->x = f1;
  std::modf((point->y - position_(1)) / resolution_(1), &f1);
  point->y = f1;
}

void GridMap::environmentToGridCoordinates(Eigen::Vector2d* point) const {
  double f1;
  std::modf(((*point)(0) - position_(0)) / resolution_(0), &f1);
  (*point)(0) = f1;
  std::modf(((*point)(1) - position_(1)) / resolution_(1), &f1);
  (*point)(1) = f1;
}

void GridMap::environmentToGridCoordinates(
    SubmapCoordinates* submap_coordinates) const {
  environmentToGridCoordinates(&(submap_coordinates->lower_left));
  environmentToGridCoordinates(&(submap_coordinates->upper_right));
  environmentToGridCoordinates(&(submap_coordinates->upper_left));
  environmentToGridCoordinates(&(submap_coordinates->lower_right));
}

void GridMap::trimSubmapCoordinatesToGrid(
    SubmapCoordinates* submap_coordinates) const {
  submap_coordinates->lower_left(0) =
      std::max(submap_coordinates->lower_left(0), 0.0);
  submap_coordinates->lower_left(1) =
      std::max(submap_coordinates->lower_left(1), 0.0);
  submap_coordinates->upper_right(0) =
      std::min(submap_coordinates->upper_right(0), length_(0) - 1.0);
  submap_coordinates->upper_right(1) =
      std::min(submap_coordinates->upper_right(1), length_(1) - 1.0);
  submap_coordinates->upper_left(0) =
      std::max(submap_coordinates->upper_left(0), 0.0);
  submap_coordinates->upper_left(1) =
      std::min(submap_coordinates->upper_left(1), length_(1) - 1.0);
  submap_coordinates->lower_right(0) =
      std::min(submap_coordinates->lower_right(0), length_(0) - 1.0);
  submap_coordinates->lower_right(1) =
      std::max(submap_coordinates->lower_right(1), 0.0);
}

double GridMap::toProbability(double x) const {
  // Set unknown occupancy (-1).
  if (x == 0)
    return -1.0 / 100;
  else
    return 1.0 - 1.0 / (1.0 + std::exp(x));
}

double GridMap::toLogOdds(double x) const {
  // Set unknown occupancy.
  if (x == -1.0 / 100)
    return 0;
  else
    return std::log(x / (1.0 - x));
}