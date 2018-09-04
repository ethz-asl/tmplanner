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

#include "tmplanner_discrete/tmplanner.h"

using namespace tmplanner;

void tmPlanner::setupPlanner() {
  elapsed_time_ = 0.0;
  control_poses_.clear();
  trajectory_.clear();

  // Set up the grid map.
  grid_map_.setMapGeometry(
      map_parameters_.width, map_parameters_.height,
      map_parameters_.resolution_x, map_parameters_.resolution_y,
      -map_parameters_.width / 2.0, -map_parameters_.height / 2.0,
      map_parameters_.upper_threshold, map_parameters_.lower_threshold,
      map_parameters_.frame_id);
  grid_map_.fillUnknown();

  // Set up the lattice for 3-D grid search.
  lattice_.createLattice(
      planning_parameters_.maximum_height, planning_parameters_.minimum_height,
      sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
      planning_parameters_.lattice_min_height_points,
      planning_parameters_.lattice_height_increment, grid_map_);
}

bool tmPlanner::isBudgetSpent() {
  if (elapsed_time_ > planning_parameters_.time_budget) {
    LOG(INFO) << "Budget spent. Exiting...";
    return true;
  } else {
    return false;
  }
}

bool tmPlanner::createInitialPlan() {
  // start_time_ = ros::Time::now().toSec();
  control_poses_.clear();
  trajectory_.clear();
  // Compute current world to vicon-sensor-body transform.
  kindr::minimal::QuatTransformation T_MAP_VSB;
  tf::poseMsgToKindr(odometry_pose_, &T_MAP_VSB);
  kindr::minimal::QuatTransformation T_W_VSB =
      map_parameters_.T_W_MAP * T_MAP_VSB;
  geometry_msgs::Pose odometry_pose_world;
  tf::poseKindrToMsg(T_W_VSB, &odometry_pose_world);
  // Check that MAV is not already at the initial position.
  control_poses_.push_back(odometry_pose_world);
  geometry_msgs::Pose target_pose;
  target_pose.position = planning_parameters_.initial_position;
  // For now, set orientation to 0.0.
  target_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
  control_poses_.push_back(target_pose);
  createPolynomialTrajectory();
  return true;
}

bool tmPlanner::createNewPlan(const geometry_msgs::Pose& current_pose) {
  control_poses_.clear();
  // STEP 1. Grid search on the lattice.
  // No next best point can be found on the lattice grid search
  if (!searchGrid(current_pose)) {
    return false;
  }
  // STEP 2. Evolutionary optimization.
  optimizePathOnHorizon();
  logger_.writeControlPoses(control_poses_);
  createPolynomialTrajectory();
  return true;
}

void tmPlanner::createLandingPlan() {
  control_poses_.clear();
  control_poses_.push_back(odometry_pose_);
  geometry_msgs::Pose landing_pose;
  landing_pose.position.x = odometry_pose_.position.x;
  landing_pose.position.y = odometry_pose_.position.y;
  landing_pose.position.z = 0.05;
  landing_pose.orientation = odometry_pose_.orientation;
  control_poses_.push_back(landing_pose);
  createPolynomialTrajectory();
}

void tmPlanner::updateMap(const geometry_msgs::PoseArray& detections_poses,
                          const geometry_msgs::Pose& odometry_pose) {
  LOG(INFO) << "Updating map at x = " << odometry_pose.position.x
            << ", y = " << odometry_pose.position.y
            << ", z = " << odometry_pose.position.z;
  grid_map_.updateMapFromPoseArray(
      detections_poses, odometry_pose, sensor_parameters_.T_IMU_CAM,
      sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
      sensor_parameters_.saturation_height,
      sensor_parameters_.intrinsics_matrix,
      sensor_parameters_.true_positive_coeffs,
      sensor_parameters_.false_negative_coeffs);
  logger_.writeMeasurementPose(odometry_pose);
  double number_of_points = grid_map_.getLength()(0) * grid_map_.getLength()(1);
  logger_.writeMapMetrics(
      grid_map_.computeEntropy(),
      (number_of_points -
       (double)grid_map_.computeNumberOfUnclassifiedPoints()) /
          number_of_points);
}

void tmPlanner::createPolynomialTrajectory() {
  CHECK_GT(control_poses_.size(), 0) << "No control poses!";

  trajectory_.clear();
  mav_trajectory_generation::Vertex::Vector path_vertices;
  mav_trajectory_generation::Vertex::Vector yaw_vertices;

  // Create the vertex lists for the planned path.
  for (std::deque<geometry_msgs::Pose>::size_type i = 0;
       i < control_poses_.size(); ++i) {
    mav_trajectory_generation::Vertex vertex(kDimensions);
    mav_trajectory_generation::Vertex yaw(1);

    if (i == 0 || i == control_poses_.size() - 1) {
      vertex.makeStartOrEnd(Eigen::Vector3d(control_poses_[i].position.x,
                                            control_poses_[i].position.y,
                                            control_poses_[i].position.z),
                            kDerivativetoOptimize);
    } else {
      vertex.addConstraint(
          mav_trajectory_generation::derivative_order::POSITION,
          Eigen::Vector3d(control_poses_[i].position.x,
                          control_poses_[i].position.y,
                          control_poses_[i].position.z));
    }
    path_vertices.push_back(vertex);

    yaw.addConstraint(mav_trajectory_generation::derivative_order::ORIENTATION,
                      tf::getYaw(control_poses_[i].orientation));
    yaw_vertices.push_back(yaw);
  }

  // Create a middle waypoint if there are only two.
  if (control_poses_.size() <= 2) {
    geometry_msgs::Pose wpa = control_poses_[0];
    geometry_msgs::Pose wpb = control_poses_[1];
    mav_trajectory_generation::Vertex midpoint_vertex(kDimensions);
    mav_trajectory_generation::Vertex midpoint_yaw(1);

    midpoint_vertex.addConstraint(
        mav_trajectory_generation::derivative_order::POSITION,
        Eigen::Vector3d(
            wpa.position.x + (wpb.position.x - wpa.position.x) / 2.0,
            wpa.position.y + (wpb.position.y - wpa.position.y) / 2.0,
            wpa.position.z + (wpb.position.z - wpa.position.z) / 2.0));
    midpoint_yaw.addConstraint(
        mav_trajectory_generation::derivative_order::ORIENTATION,
        tf::getYaw(wpa.orientation) +
            (tf::getYaw(wpb.orientation) - tf::getYaw(wpa.orientation)) / 2.0);
    path_vertices.insert(path_vertices.begin() + 1, midpoint_vertex);
    yaw_vertices.insert(yaw_vertices.begin() + 1, midpoint_yaw);
  }

  // Optimize polynomial trajectory.
  std::vector<double> segment_times;
  mav_trajectory_generation::Trajectory path_trajectory, yaw_trajectory;
  // Position.
  segment_times = estimateSegmentTimes(
      path_vertices, planning_parameters_.reference_speed,
      planning_parameters_.reference_acceleration);
  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      opt(kDimensions);
  opt.setupFromVertices(path_vertices, segment_times, kDerivativetoOptimize);
  opt.solveLinear();
  opt.getTrajectory(&path_trajectory);
  // Yaw.
  mav_trajectory_generation::PolynomialOptimization<kPolynomialCoefficients>
      yaw_opt(1);
  yaw_opt.setupFromVertices(yaw_vertices, segment_times, kDerivativetoOptimize);
  yaw_opt.solveLinear();
  yaw_opt.getTrajectory(&yaw_trajectory);
  path_trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory,
                                                     &trajectory_);
}

bool tmPlanner::searchGrid(const geometry_msgs::Pose& initial_pose) {
  // Initialize variables based on the current map state.
  control_poses_.push_back(initial_pose);
  auto previous_pose = initial_pose;
  grid_map::GridMap simulated_grid_map = grid_map_;

  // Perform the grid search.
  while (control_poses_.size() < planning_parameters_.control_poses) {
    geometry_msgs::Pose best_pose;
    // Find the next best point on the lattice.
    if (optimization_parameters_.optimization_objective ==
        OPTIMIZATION_OBJECTIVE_INFORMATION) {
      best_pose =
          findNextBestInformationGridPoint(simulated_grid_map, previous_pose);
    } else if (optimization_parameters_.optimization_objective ==
               OPTIMIZATION_OBJECTIVE_CLASSIFICATION) {
      best_pose = findNextBestClassificationGridPoint(simulated_grid_map,
                                                      previous_pose);
    } else if (optimization_parameters_.optimization_objective ==
               OPTIMIZATION_OBJECTIVE_TIMEVARYING) {
      // TODO. Time-varying objective which scales with mission time.
    }
    // No next best point can be found (out-of-bounds result).
    if (best_pose.position.x == 1000.0 && best_pose.position.y == 1000.0 &&
        best_pose.position.z == -1.0) {
      return false;
    }
    // Simulate a measurement at this point.
    simulated_grid_map.predictMapUpdate(
        best_pose, sensor_parameters_.T_IMU_CAM, sensor_parameters_.fov_angle_x,
        sensor_parameters_.fov_angle_y, sensor_parameters_.saturation_height,
        sensor_parameters_.true_positive_coeffs,
        sensor_parameters_.false_negative_coeffs);
    control_poses_.push_back(best_pose);
    previous_pose = best_pose;
    LOG(INFO) << "Map entropy: " << simulated_grid_map.computeEntropy();
  }
  return true;
}

geometry_msgs::Pose tmPlanner::findNextBestInformationGridPoint(
    const grid_map::GridMap& simulated_grid_map,
    const geometry_msgs::Pose& previous_pose) {
  geometry_msgs::Pose evaluated_pose;
  evaluated_pose.orientation =
      tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);

  const auto entropy_prev = simulated_grid_map.computeEntropy();

  // Initialize the best solution found so far.
  double maximum_objective = 0.0;
  geometry_msgs::Pose best_pose;
  // Simulate measurement for candidate points in lattice and
  // calculate the informative objective.
  for (const auto& evaluated_point : lattice_.getLatticePoints()) {
    evaluated_pose.position = evaluated_point;
    // Create a copy of the grid map for evaluating each lattice point.
    grid_map::GridMap evaluated_grid_map = simulated_grid_map;
    evaluated_grid_map.predictMapUpdate(
        evaluated_pose, sensor_parameters_.T_IMU_CAM,
        sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
        sensor_parameters_.saturation_height,
        sensor_parameters_.true_positive_coeffs,
        sensor_parameters_.false_negative_coeffs);

    // Calculate gain (reduction in map entropy).
    auto gain = entropy_prev - evaluated_grid_map.computeEntropy();
    // Calculate cost (time from previous viewpoint), assuming constant
    // speed. Limit based on measurement frequency.
    auto cost = std::max(getDistanceBetweenPoints(previous_pose.position,
                                                  evaluated_pose.position) /
                             planning_parameters_.reference_speed,
                         1.0 / sensor_parameters_.measurement_frequency);
    // Calculate the objective: rate of information gain.
    double objective = gain / cost;

    if (objective > maximum_objective) {
      maximum_objective = objective;
      best_pose = evaluated_pose;
    }
  }
  LOG(INFO) << "Best position: " << best_pose.position;
  LOG(INFO) << "Maximum objective: " << maximum_objective;
  // Return out-of-bounds value if planning is complete.
  if (maximum_objective <= 0.0) {
    LOG(INFO)
        << "Could not find the next best lattice point! Planning is complete.";
    best_pose.position.x = 1000.0;
    best_pose.position.y = 1000.0;
    best_pose.position.z = -1.0;
  }
  return best_pose;
}

geometry_msgs::Pose tmPlanner::findNextBestClassificationGridPoint(
    const grid_map::GridMap& simulated_grid_map,
    const geometry_msgs::Pose& previous_pose) {
  geometry_msgs::Pose evaluated_pose;
  evaluated_pose.orientation =
      tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);

  const auto unclassified_points_prev =
      simulated_grid_map.computeNumberOfUnclassifiedPoints();

  // Initialize the best solution found so far.
  double maximum_objective = 0.0;
  geometry_msgs::Pose best_pose;
  // Simulate measurement for candidate points in lattice and
  // calculate the informative objective.
  for (const auto& evaluated_point : lattice_.getLatticePoints()) {
    evaluated_pose.position = evaluated_point;
    // Create a copy of the grid map for evaluating each lattice point.
    grid_map::GridMap evaluated_grid_map = simulated_grid_map;
    evaluated_grid_map.predictMapUpdate(
        evaluated_pose, sensor_parameters_.T_IMU_CAM,
        sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
        sensor_parameters_.saturation_height,
        sensor_parameters_.true_positive_coeffs,
        sensor_parameters_.false_negative_coeffs);

    // Calculate gain (reduction in map entropy).
    auto gain = unclassified_points_prev -
                evaluated_grid_map.computeNumberOfUnclassifiedPoints();
    // Calculate cost (time from previous viewpoint), assuming constant
    // speed. Limit based on measurement frequency.
    auto cost = std::max(getDistanceBetweenPoints(previous_pose.position,
                                                  evaluated_pose.position) /
                             planning_parameters_.reference_speed,
                         1.0 / sensor_parameters_.measurement_frequency);
    // Calculate the objective: rate of information gain.
    double objective = gain / cost;

    if (objective > maximum_objective) {
      maximum_objective = objective;
      best_pose = evaluated_pose;
    }
  }
  LOG(INFO) << "Best position: " << best_pose.position;
  LOG(INFO) << "Maximum objective: " << maximum_objective;
  // Return out-of-bounds value if planning is complete.
  if (maximum_objective <= 0.0) {
    LOG(INFO)
        << "Could not find the next best lattice point! Planning is complete.";
    best_pose.position.x = 1000.0;
    best_pose.position.y = 1000.0;
    best_pose.position.z = -1.0;
  }
  return best_pose;
}

void tmPlanner::optimizePathOnHorizon() {
  CHECK_GT(control_poses_.size(), 0) << "No control poses to optimize!";

  // Use the CMA-ES.
  if (optimization_parameters_.optimization_method == "cmaes") {
    // Initialize optimization parameters.
    control_poses_.pop_front();
    int dim = control_poses_.size() * kDimensions;
    std::vector<double> x0;
    for (auto pose : control_poses_) {
      x0.push_back(pose.position.x);
      x0.push_back(pose.position.y);
      x0.push_back(pose.position.z);
    }

    double lbounds[dim], ubounds[dim];
    for (size_t i = 0; i < dim; i += 3) {
      lbounds[i] =
          -(grid_map_.getLength()(0) * grid_map_.getResolution()(0)) / 2.0;
      ubounds[i] =
          grid_map_.getLength()(0) * grid_map_.getResolution()(0) / 2.0;
      lbounds[i + 1] =
          -(grid_map_.getLength()(1) * grid_map_.getResolution()(1)) / 2.0;
      ubounds[i + 1] =
          grid_map_.getLength()(1) * grid_map_.getResolution()(1) / 2.0;
      lbounds[i + 2] = planning_parameters_.minimum_height;
      ubounds[i + 2] = planning_parameters_.maximum_height;
    }

    // Perform the optimization.
    libcmaes::FitFunc optimize_control_poses =
        std::bind(&tmPlanner::optimizeControlPoses, this, std::placeholders::_1,
                  std::placeholders::_2);
    libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> geno(lbounds, ubounds, dim);
    libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>
        cmaparams(dim, &x0.front(), optimization_parameters_.cmaes_step_size,
                  optimization_parameters_.cmaes_offsprings, 0, geno);
    cmaparams.set_max_fevals(optimization_parameters_.cmaes_maximum_fevals);
    libcmaes::CMASolutions cmasols =
        libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>(
            optimize_control_poses, cmaparams);

    // Write best solution.
    const Eigen::VectorXd x_best =
        geno.pheno(cmasols.get_best_seen_candidate().get_x_dvec());
    control_poses_.clear();
    // Compute world to vicon-sensor-body transform for trajectory commands.
    kindr::minimal::QuatTransformation T_MAP_VSB;
    tf::poseMsgToKindr(odometry_pose_, &T_MAP_VSB);
    kindr::minimal::QuatTransformation T_W_VSB =
        map_parameters_.T_W_MAP * T_MAP_VSB;
    geometry_msgs::Pose pose_world;
    tf::poseKindrToMsg(T_W_VSB, &pose_world);
    control_poses_.push_back(pose_world);
    geometry_msgs::Pose pose;
    for (size_t i = 0; i < dim; i += 3) {
      pose.position.x = x_best[i];
      pose.position.y = x_best[i + 1];
      pose.position.z = x_best[i + 2];
      pose.orientation =
          tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);
      tf::poseMsgToKindr(pose, &T_MAP_VSB);
      T_W_VSB = map_parameters_.T_W_MAP * T_MAP_VSB;
      tf::poseKindrToMsg(T_W_VSB, &pose_world);
      control_poses_.push_back(pose_world);
    }
  }
}

double tmPlanner::optimizeControlPoses(const double* x, const int N) {
  // Add current pose as first control pose for the polynomial.
  control_poses_.clear();
  control_poses_.push_back(odometry_pose_);

  // Construct the list of control poses.
  for (size_t i = 0; i < N; i += 3) {
    geometry_msgs::Pose pose;
    pose.position.x = x[i];
    pose.position.y = x[i + 1];
    pose.position.z = x[i + 2];
    pose.orientation =
        tf::createQuaternionMsgFromYaw(planning_parameters_.reference_yaw);
    control_poses_.push_back(pose);
  }

  auto gain = 0.0;
  double info_prev;
  auto simulated_grid_map = grid_map_;
  if (optimization_parameters_.optimization_objective ==
      OPTIMIZATION_OBJECTIVE_INFORMATION) {
    info_prev = simulated_grid_map.computeEntropy();
  } else if (optimization_parameters_.optimization_objective ==
             OPTIMIZATION_OBJECTIVE_CLASSIFICATION) {
    info_prev = simulated_grid_map.computeNumberOfUnclassifiedPoints();
  }

  createPolynomialTrajectory();
  mav_msgs::EigenTrajectoryPoint::Vector measurement_states;
  mav_trajectory_generation::sampleWholeTrajectory(
      trajectory_, 1.0 / sensor_parameters_.measurement_frequency,
      &measurement_states);
  // Discard paths that are too long.
  if (measurement_states.size() > 10) {
    return 1000;
  }

  // Perform map update predictions.
  for (const auto& measurement_state : measurement_states) {
    geometry_msgs::PoseStamped measurement_pose;
    // Discard measurement locations that are out-of-bounds.
    if ((measurement_pose.pose.position.x <
         -(grid_map_.getLength()(0) * grid_map_.getResolution()(0)) / 2.0) ||
        (measurement_pose.pose.position.x >
         (grid_map_.getLength()(0) * grid_map_.getResolution()(0)) / 2.0) ||
        (measurement_pose.pose.position.y <
         -(grid_map_.getLength()(1) * grid_map_.getResolution()(1)) / 2.0) ||
        (measurement_pose.pose.position.y <
         -(grid_map_.getLength()(1) * grid_map_.getResolution()(1)) / 2.0) ||
        measurement_pose.pose.position.z <
            planning_parameters_.minimum_height ||
        measurement_pose.pose.position.z >
            planning_parameters_.maximum_height) {
      return 1000;
    }
    mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(measurement_state,
                                                     &measurement_pose);
    simulated_grid_map.predictMapUpdate(
        measurement_pose.pose, sensor_parameters_.T_IMU_CAM,
        sensor_parameters_.fov_angle_x, sensor_parameters_.fov_angle_y,
        sensor_parameters_.saturation_height,
        sensor_parameters_.true_positive_coeffs,
        sensor_parameters_.false_negative_coeffs);
  }

  if (optimization_parameters_.optimization_objective ==
      OPTIMIZATION_OBJECTIVE_INFORMATION) {
    gain = info_prev - simulated_grid_map.computeEntropy();
  } else if (optimization_parameters_.optimization_objective ==
             OPTIMIZATION_OBJECTIVE_CLASSIFICATION) {
    gain = info_prev - simulated_grid_map.computeNumberOfUnclassifiedPoints();
  }

  auto cost = trajectory_.getMaxTime();
  LOG(INFO) << "Objective = " << -gain / cost;
  return -gain / cost;
}

bool tmPlanner::arePointsEqual(const geometry_msgs::Point& p1,
                               const geometry_msgs::Point& p2,
                               const double& tol) {
  return (fabs(p1.x - p2.x) < tol) && (fabs(p1.y - p2.y) < tol) &&
         (fabs(p1.z - p2.z) < tol);
}

double tmPlanner::getDistanceBetweenPoints(const geometry_msgs::Point& p1,
                                           const geometry_msgs::Point& p2) {
  return sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0) +
              pow(p1.z - p2.z, 2.0));
}