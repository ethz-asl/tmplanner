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

#include "tmplanner_continuous/grid_map.h"

using namespace grid_map;

void GridMap::setMapGeometry(const double& width, const double& height,
                             const double& resolution_x,
                             const double& resolution_y,
                             const double& position_x, const double& position_y,
                             const std::string& frame_id) {
  CHECK_GT(resolution_x, 0) << "Error in map resolution value in x.";
  CHECK_GT(resolution_y, 0) << "Error in map resolution value in y.";
  CHECK_GE(width, 0) << "Error in map width value.";
  CHECK_GE(height, 0) << "Error in map height value.";

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

void GridMap::fillUnknown() {
  data_.setOnes(length_(0), length_(1));
  data_ = data_ * 0.5;
}

void GridMap::fillMapData(const Eigen::MatrixXd& data) {
  CHECK_EQ(length_(0), data.rows())
      << "Data dimensions don't match map dimensions in x.";
  CHECK_EQ(length_(1), data.cols())
      << "Data dimensions don't match map dimensions in y.";
  data_ = data;
}

void GridMap::computeCovariance(
    const std::string cov_fun, const std::vector<double> log_hyperparams_vector,
    const double& resolution_x_predict, const double& resolution_y_predict) {
  if (cov_fun == "matern3") {
    gp::CovMatern3iso<2, double> cov;
    Eigen::Vector2d log_hyperparams;
    log_hyperparams << log_hyperparams_vector[0],
        log_hyperparams_vector[1];  // Length scale, signal variance squared
    cov.setLogHyperparams(log_hyperparams);

    // Create the inputs.
    // Mapping resolution.
    const unsigned int num_inputs = length_(0) * length_(1);
    gp::CovMatern3iso<2, double>::Inputs X(2, num_inputs);
    size_t counter = 0;
    for (size_t i = 1; i <= length_(0); ++i) {
      for (size_t j = 1; j <= length_(1); ++j) {
        X(0, counter) = i;
        X(1, counter) = j;
        counter++;
      }
    }

    // Predictive resolution.
    const size_t length_x_predict =
        length_(0) * (resolution_(0) / resolution_x_predict);
    const size_t length_y_predict =
        length_(1) * (resolution_(1) / resolution_y_predict);

    const size_t num_inputs_predict = length_x_predict * length_y_predict;
    gp::CovMatern3iso<2, double>::Inputs X_predict(2, num_inputs_predict);
    counter = 0;
    for (size_t i = 1; i <= length_x_predict; ++i) {
      for (size_t j = 1; j <= length_y_predict; ++j) {
        X_predict(0, counter) = i;
        X_predict(1, counter) = j;
        counter++;
      }
    }

    // Compute the covariance using the kernel.
    Eigen::MatrixXd K, Kss, Kst;
    K.resize(num_inputs, num_inputs);
    Kss.resize(num_inputs_predict, num_inputs_predict);
    Kst.resize(num_inputs_predict, num_inputs);

    cov.getCovarianceMatrix(X, &K);
    // Add noise variance hyperparameter from likelihood function.
    K = K + Eigen::MatrixXd::Identity(K.rows(), K.cols()) * exp(2 * 0.35);
    cov.getCovarianceMatrix(X_predict, &Kss);
    cov.getCovarianceMatrix(X_predict, X, &Kst);

    covariance_ = Kss - Kst * K.inverse() * Kst.transpose();
  }
}

double GridMap::computeCovarianceTrace() const { return covariance_.trace(); }

double GridMap::computeCovarianceTrace(const double& lower_threshold) const {
  double cov_trace = 0.0;
  Eigen::MatrixXd P_diag = covariance_.diagonal();
  Eigen::Map<const Eigen::MatrixXd> P(P_diag.data(), data_.rows(),
                                      data_.cols());
  for (size_t j = 0; j < data_.cols(); ++j) {
    for (size_t i = 0; i < data_.rows(); ++i) {
      if (data_(i, j) > lower_threshold) {
        cov_trace += P(i, j);
      }
    }
  }
  return cov_trace;
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

void GridMap::updateMapFromImage(
    const cv_bridge::CvImagePtr& cv_image_ptr,
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const cv::Mat& intrinsics_matrix, const double& sensor_coefficient_A,
    const double& sensor_coefficient_B) {
  // Obtain world to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute world to camera transform.
  kindr::minimal::QuatTransformation T_MAP_CAM = T_MAP_IMU * T_IMU_CAM;
  geometry_msgs::Pose camera_pose;
  tf::poseKindrToMsg(T_MAP_CAM, &camera_pose);

  // Read the image to project to the map.
  const cv::Mat image = cv_image_ptr->image;

  // Extract the RGB color channels.
  cv::Mat channels[3];
  // Extract the LAB channels.
  // cv::cvtColor(image, image, cv::COLOR_BGR2Lab);
  // Extract the HSV channels.
  cv::cvtColor(image, image, cv::COLOR_BGR2HSV);
  split(image, channels);

  // Calculate the corner co-ordinates of the observed submap.
  SubmapCoordinates submap_coordinates;
  Eigen::Vector4d p1;
  p1 << 0.0, 0.0, 1.0, 1.0;
  Eigen::Vector4d p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.upper_left = p2.head(2);
  p1 << 0.0, (image.rows - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_left = p2.head(2);
  p1 << (image.cols - 1.0), (image.rows - 1.0), 1.0, 1.0;
  p2 = projectPixelToGround(intrinsics_matrix, T_MAP_CAM, p1);
  submap_coordinates.lower_right = p2.head(2);
  p1 << (image.cols - 1.0), 0.0, 1.0, 1.0;
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
  Eigen::MatrixXd submap_counter, submap_data;
  submap_counter.setZero(submap_size_y, submap_size_x);
  submap_data.setZero(submap_size_y, submap_size_x);

  for (size_t i = 0; i < image.rows; ++i) {
    for (size_t j = 0; j < image.cols; ++j) {
      // Apply the homography transform to each image pixel.
      Eigen::Vector4d image_point;
      image_point << j, i, 1.0, 1.0;
      Eigen::Vector4d transformed_point =
          projectPixelToGround(intrinsics_matrix, T_MAP_CAM, image_point);
      Eigen::Vector2d grid_point = transformed_point.head(2);
      environmentToGridCoordinates(&grid_point);
      // Skip measurement: out-of-bounds.
      if (grid_point(0) - x_min < 0 ||
          grid_point(0) - x_min > (submap_size_x - 1) ||
          grid_point(1) - y_min < 0 ||
          grid_point(1) - y_min > (submap_size_y - 1) || grid_point(0) < 0 ||
          grid_point(1) < 0 || grid_point(0) > (length_(0) - 1) ||
          grid_point(1) > (length_(1) - 1)) {
        continue;
      }
      submap_counter(grid_point(1) - y_min, grid_point(0) - x_min)++;
      submap_data(grid_point(1) - y_min, grid_point(0) - x_min) +=
          (unsigned int)channels[0].at<uchar>(i, j);
    }
  }

  // Identify valid measuremsents and their indices.
  Eigen::VectorXd measurements, measurement_indices;
  measurements.setZero(submap_size_x * submap_size_y);
  measurement_indices.setZero(submap_size_x * submap_size_y);
  size_t counter = 0;
  for (size_t i = 0; i < submap_data.rows(); ++i) {
    for (size_t j = 0; j < submap_data.cols(); ++j) {
      // Skip measurement: not enough pixels in cell.
      if (submap_counter(i, j) < 100) {
        continue;
      }
      int row_coordinate = y_min + i;
      int col_coordinate = x_min + j;
      int index = row_coordinate + data_.rows() * col_coordinate;
      // Scale measurement to 0-1.
      measurements(counter) =
          (submap_data(i, j) / submap_counter(i, j)) / 255.0;
      measurement_indices(counter) = index;
      counter++;
    }
  }

  // No valid measurements in this image.
  if (counter == 0) {
    return;
  }
  // Compute matrices for KF update.
  const double var = computeSensorNoiseVariance(
      camera_pose.position.z, sensor_coefficient_A, sensor_coefficient_B);
  const Eigen::SparseMatrix<double> H =
      constructMeasurementModel(measurement_indices.head(counter));
  KFUpdate(measurements.head(counter), var, H);
}

void GridMap::predictMapUpdate(
    const geometry_msgs::Pose& mav_pose,
    const kindr::minimal::QuatTransformation& T_IMU_CAM,
    const double& sensor_fov_angle_x, const double& sensor_fov_angle_y,
    const double& sensor_coefficient_A, const double& sensor_coefficient_B) {
  // Obtain world to IMU (vicon-sensor-body) transform.
  kindr::minimal::QuatTransformation T_MAP_IMU;
  tf::poseMsgToKindr(mav_pose, &T_MAP_IMU);
  // Compute world to camera transform.
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
  const size_t submap_size_x = submap_coordinates_grid.upper_right(0) -
                               submap_coordinates_grid.lower_left(0) + 1;
  const size_t submap_size_y = submap_coordinates_grid.upper_right(1) -
                               submap_coordinates_grid.lower_left(1) + 1;

  // Get indices of the observed part of the environment (predicted submap).
  Eigen::VectorXd measurement_indices(submap_size_x * submap_size_y);
  size_t counter = 0;
  for (size_t i = submap_coordinates_grid.lower_left(0);
       i <= submap_coordinates_grid.upper_right(0); ++i) {
    for (size_t j = submap_coordinates_grid.lower_left(1);
         j <= submap_coordinates_grid.upper_right(1); ++j) {
      int row_coordinate = submap_coordinates_grid.upper_right(1) - j +
                           submap_coordinates_grid.lower_left(1);
      int col_coordinate = i;
      int index = row_coordinate + data_.rows() * col_coordinate;
      measurement_indices(counter) = index;
      counter++;
    }
  }

  // Compute matrices for KF update.
  const double var = computeSensorNoiseVariance(
      camera_pose.position.z, sensor_coefficient_A, sensor_coefficient_B);
  const Eigen::SparseMatrix<double> H =
      constructMeasurementModel(measurement_indices);
  // Perform update for covariance only.
  KFUpdate(var, H);
}

void GridMap::KFUpdate(const Eigen::VectorXd& z, const double& var,
                       const Eigen::SparseMatrix<double>& H) {
  // Compute the innovation. NB: Column-major ordering.
  Eigen::Map<const Eigen::MatrixXd> x(data_.data(), data_.rows() * data_.cols(),
                                      1);
  Eigen::MatrixXd v = z - H * x;
  Eigen::MatrixXd x_new;
  // Kalman filter update.
  Eigen::MatrixXd PHt = covariance_ * H.transpose();
  Eigen::MatrixXd S =
      Eigen::MatrixXd(H * PHt) +
      Eigen::MatrixXd((var * Eigen::MatrixXd::Ones(H.rows(), 1)).asDiagonal());
  const Eigen::LLT<Eigen::MatrixXd> llt_of_S(S);
  // Check if S is positive definite.
  if (llt_of_S.info() == Eigen::NumericalIssue) {
    x_new = x + PHt * S.inverse() * v;
    covariance_ = covariance_ - PHt * S.inverse() * H * covariance_;
  } else {
    Eigen::MatrixXd Wc = llt_of_S.matrixU().solve<Eigen::OnTheRight>(PHt);
    Eigen::MatrixXd W =
        llt_of_S.matrixU().transpose().solve<Eigen::OnTheRight>(Wc);
    logger_.writeKFUpdateData(z, H, var, covariance_, data_);
    x_new = x + W * v;
    covariance_ = covariance_ - Wc * Wc.transpose();
  }
  x_new.resize(data_.rows(), data_.cols());
  data_ = x_new;
}

void GridMap::KFUpdate(const double& var,
                       const Eigen::SparseMatrix<double>& H) {
  Eigen::LLT<Eigen::MatrixXd> llt_of_S(
      Eigen::MatrixXd(H * covariance_ * H.transpose()) +
      Eigen::MatrixXd((var * Eigen::MatrixXd::Ones(H.rows(), 1)).asDiagonal()));
  // Check if S is positive definite.
  if (llt_of_S.info() == Eigen::NumericalIssue) {
    covariance_ =
        covariance_ -
        covariance_ * H.transpose() *
            (Eigen::MatrixXd(H * covariance_ * H.transpose()) +
             Eigen::MatrixXd(
                 (var * Eigen::MatrixXd::Ones(H.rows(), 1)).asDiagonal()))
                .inverse() *
            H * covariance_;
  } else {
    Eigen::MatrixXd Wc = llt_of_S.matrixU().solve<Eigen::OnTheRight>(
        covariance_ * H.transpose());
    covariance_ = covariance_ - Wc * Wc.transpose();
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

Eigen::SparseMatrix<double> GridMap::constructMeasurementModel(
    const Eigen::VectorXd& measurement_indices) const {
  Eigen::SparseMatrix<double> H(measurement_indices.rows(),
                                data_.rows() * data_.cols());
  // Identify and index the states corresponding to a measurement.
  // TODO: - Multiresolution measurements?
  for (size_t i = 0; i < measurement_indices.rows(); ++i) {
    H.insert(i, measurement_indices(i)) = 1.0;
  }
  return H;
}

double GridMap::computeSensorNoiseVariance(
    const double& camera_height, const double& sensor_coefficient_A,
    const double& sensor_coefficient_B) const {
  return sensor_coefficient_A *
         (1 - exp(-sensor_coefficient_B * camera_height));
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