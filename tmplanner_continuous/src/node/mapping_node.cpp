/*
 * Copyright (c) 2018, Marija Popovic, ASL, ETH Zurich, Switzerland
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

#include "tmplanner_continuous/mapping_node.h"

using namespace tmplanner;

MappingNode::MappingNode(const ros::NodeHandle& nh,
                         const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), got_odometry_(false) {
  loadParameters();
  tmplanner_.setupPlanner();

  // Set up ROS comms.
  odometry_sub_ =
      nh_.subscribe("odometry", 1, &MappingNode::odometryCallback, this);
  image_sub_ = nh_.subscribe("image", 1, &MappingNode::imageCallback, this);
}

void MappingNode::loadParameters() {
  double width, height, resolution_x, resolution_y;
  std::vector<double> log_hyperparams;
  std::string frame_id, cov_fun;
  MapParameters map_parameters;
  CHECK(nh_.getParam("width", width) && nh_.getParam("height", height) &&
        nh_.getParam("map_resolution_x", resolution_x) &&
        nh_.getParam("map_resolution_y", resolution_y) &&
        nh_.getParam("frame_id", frame_id))
      << "Error loading map parameters!";
  CHECK(nh_.getParam("log_hyperparams", log_hyperparams) &&
        nh_.getParam("cov_fun", cov_fun) && cov_fun == "matern3")
      << "Error loading GP params!";
  if (cov_fun == "matern3") {
    CHECK_EQ(log_hyperparams.size(), 2);
  }
  map_parameters.width = width;
  map_parameters.height = height;
  map_parameters.resolution_x = resolution_x;
  map_parameters.resolution_y = resolution_y;
  map_parameters.frame_id = frame_id;
  map_parameters.cov_fun = cov_fun;
  map_parameters.log_hyperparams = log_hyperparams;
  tmplanner_.setMapParameters(map_parameters);

  int control_poses, lattice_min_height_points;
  double maximum_height, minimum_height, reference_speed,
      reference_acceleration, reference_yaw, time_budget,
      lattice_height_increment, lower_threshold;
  geometry_msgs::Point initial_position;
  bool use_threshold;
  PlanningParameters planning_parameters;
  CHECK(nh_.getParam("control_poses", control_poses) &&
        nh_.getParam("maximum_height", maximum_height) &&
        nh_.getParam("minimum_height", minimum_height) &&
        nh_.getParam("lattice_min_height_points", lattice_min_height_points) &&
        nh_.getParam("lattice_height_increment", lattice_height_increment))
      << "Error loading planning parameters!";
  CHECK(nh_.getParam("reference_speed", reference_speed) &&
        nh_.getParam("reference_acceleration", reference_acceleration) &&
        nh_.getParam("time_budget", time_budget) &&
        nh_.getParam("reference_yaw", reference_yaw))
      << "Error loading planning parameters!";
  CHECK(nh_.getParam("initial_pos_x", initial_position.x) &&
        nh_.getParam("initial_pos_y", initial_position.y) &&
        nh_.getParam("initial_pos_z", initial_position.z))
      << "Error loading planning parameters!";
  CHECK(nh_.getParam("use_threshold", use_threshold) &&
        nh_.getParam("lower_threshold", lower_threshold))
      << "Error loading planning parameters!";
  CHECK_GT(control_poses, 1) << "Invalid number of polynomial control poses "
                                "- must be between 1 and 10.";
  CHECK_LT(control_poses, 10) << "Invalid number of polynomial control poses "
                                 "- must be between 1 and 10.";
  CHECK_GE(initial_position.x, -width / 2.0)
      << "Initial position not within environment range!";
  CHECK_GE(initial_position.y, -height / 2.0)
      << "Initial position not within environment range!";
  CHECK_GE(initial_position.z, minimum_height)
      << "Initial position not within environment range!";
  CHECK_LE(initial_position.x, width / 2.0)
      << "Initial position not within environment range!";
  CHECK_LE(initial_position.y, height / 2.0)
      << "Initial position not within environment range!";
  CHECK_LE(initial_position.z, maximum_height)
      << "Initial position not within environment range!";
  planning_parameters.control_poses = control_poses;
  planning_parameters.initial_position = initial_position;
  planning_parameters.maximum_height = maximum_height;
  planning_parameters.minimum_height = minimum_height;
  planning_parameters.reference_speed = reference_speed;
  planning_parameters.reference_acceleration = reference_acceleration;
  planning_parameters.reference_yaw = reference_yaw;
  planning_parameters.time_budget = time_budget;
  planning_parameters.lattice_min_height_points = lattice_min_height_points;
  planning_parameters.lattice_height_increment = lattice_height_increment;
  planning_parameters.use_threshold = use_threshold;
  planning_parameters.lower_threshold = lower_threshold;
  tmplanner_.setPlanningParameters(planning_parameters);

  double measurement_frequency, coefficient_A, coefficient_B, fov_angle_x,
      fov_angle_y;
  SensorParameters sensor_parameters;
  XmlRpc::XmlRpcValue T_IMU_CAM_xml;
  kindr::minimal::QuatTransformation T_IMU_CAM;
  std::vector<double> intrinsics_matrix_vector;
  CHECK(nh_.getParam("T_IMU_CAM", T_IMU_CAM_xml) &&
        nh_.getParam("intrinsics_matrix", intrinsics_matrix_vector))
      << "Could not find camera calibration parameters!";
  kindr::minimal::xmlRpcToKindr(T_IMU_CAM_xml, &T_IMU_CAM);
  cv::Mat intrinsics_matrix =
      cv::Mat(3, 3, CV_64FC1, intrinsics_matrix_vector.data());
  CHECK(nh_.getParam("measurement_frequency", measurement_frequency) &&
        nh_.getParam("coefficient_A", coefficient_A) &&
        nh_.getParam("coefficient_B", coefficient_B) &&
        nh_.getParam("fov_angle_x", fov_angle_x) &&
        nh_.getParam("fov_angle_y", fov_angle_y))
      << "Error loading sensor parameters!";
  sensor_parameters.measurement_frequency = measurement_frequency;
  sensor_parameters.coefficient_A = coefficient_A;
  sensor_parameters.coefficient_B = coefficient_B;
  sensor_parameters.fov_angle_x = fov_angle_x;
  sensor_parameters.fov_angle_y = fov_angle_y;
  sensor_parameters.T_IMU_CAM = T_IMU_CAM;
  sensor_parameters.intrinsics_matrix = intrinsics_matrix;
  tmplanner_.setSensorParameters(sensor_parameters);

  std::string optimization_method;
  double cmaes_step_size;
  int cmaes_maximum_fevals, cmaes_offsprings;
  OptimizationParameters optimization_parameters;
  CHECK(nh_.getParam("optimization_method", optimization_method) &&
        nh_.getParam("cmaes_step_size", cmaes_step_size) &&
        nh_.getParam("cmaes_maximum_fevals", cmaes_maximum_fevals) &&
        nh_.getParam("cmaes_offsprings", cmaes_offsprings))
      << "Error loading optimization parameters!";

  optimization_parameters.optimization_method = optimization_method;
  optimization_parameters.cmaes_step_size = cmaes_step_size;
  optimization_parameters.cmaes_maximum_fevals = cmaes_maximum_fevals;
  optimization_parameters.cmaes_offsprings = cmaes_offsprings;
  tmplanner_.setOptimizationParameters(optimization_parameters);

  XmlRpc::XmlRpcValue T_W_MAP_xml;
  CHECK(nh_.getParam("T_W_MAP", T_W_MAP_xml))
      << "Could not find world to map transform!";
  kindr::minimal::xmlRpcToKindr(T_W_MAP_xml, &T_W_MAP_);
}

void MappingNode::odometryCallback(
    const nav_msgs::OdometryConstPtr& odometry_message) {
  // Get world to vicon-sensor-body transform.
  kindr::minimal::QuatTransformation T_W_VSB;
  tf::poseMsgToKindr(odometry_message->pose.pose, &T_W_VSB);
  // Compute map to vicon-sensor-body transform.
  kindr::minimal::QuatTransformation T_MAP_VSB = T_W_MAP_.inverse() * T_W_VSB;
  geometry_msgs::Pose odometry_pose;
  tf::poseKindrToMsg(T_MAP_VSB, &odometry_pose);

  tmplanner_.setOdometryPose(odometry_pose);
  odometry_pose_ = odometry_pose;
  odom_time_ = odometry_message->header.stamp;

  if (!got_odometry_) {
    got_odometry_ = true;
  }
}

void MappingNode::imageCallback(
    const sensor_msgs::ImageConstPtr& image_message) {
  geometry_msgs::Pose odom_pose = odometry_pose_;
  ros::Time odom_time = odom_time_;
  if (abs(image_message->header.stamp.toNSec() - odom_time.toNSec()) >
      0.5 * pow(10, 9.0)) {
    LOG(INFO) << "More than 0.5s delay between image and odometry messages! "
                 "Skipping measurement...";
    return;
  }
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr =
        cv_bridge::toCvCopy(image_message, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    LOG(FATAL) << "cv_bridge exception: ", e.what();
  }
  tmplanner_.updateMap(cv_ptr, odom_pose);
}

int main(int argc, char** argv) {
  // Start the logging.
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  // Initialize ROS, start node.
  ros::init(argc, argv, "mapping_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  MappingNode mapping_node(nh, nh_private);
  // Receive callbacks in queue.
  ros::spin();
  return 0;
}