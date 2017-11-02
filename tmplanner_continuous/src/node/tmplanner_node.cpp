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

#include "tmplanner_node.h"

using namespace tmplanner;

tmPlannerNode::tmPlannerNode(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      got_odometry_(false),
      do_map_updates_(false) {
  loadParameters();
  tmplanner_.setupPlanner();
  LOG(INFO) << "Ready to start IPP!";

  // Set up ROS comms.
  odometry_sub_ =
      nh_.subscribe("odometry", 1, &tmPlannerNode::odometryCallback, this);
  image_sub_ = nh_.subscribe("image", 1, &tmPlannerNode::imageCallback, this);
  path_segments_pub_ =
      nh_.advertise<planning_msgs::PolynomialTrajectory4D>("path_segments", 1);
  polynomial_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
      "polynomial_markers", 1, true);
  path_points_marker_pub_ =
      nh_.advertise<visualization_msgs::Marker>("path_points_marker", 0);
  start_planning_srv_ = nh_.advertiseService(
      "start_planning", &tmPlannerNode::startPlanningCallback, this);
  land_srv_ =
      nh_.advertiseService("land", &tmPlannerNode::landCallback, this);
}

void tmPlannerNode::loadParameters() {
  double width, height, resolution_x, resolution_y;
  std::vector<double> log_hyperparams;
  std::string cov_fun;
  MapParameters map_parameters;
  CHECK(nh_.getParam("width", width) && nh_.getParam("height", height) &&
        nh_.getParam("map_resolution_x", resolution_x) &&
        nh_.getParam("map_resolution_y", resolution_y) &&
        nh_.getParam("frame_id", map_frame_id_))
      << "Error loading map parameters!";
  XmlRpc::XmlRpcValue T_W_MAP_xml;
  CHECK(nh_.getParam("T_W_MAP", T_W_MAP_xml))
      << "Could not find world to map transform!";
  kindr::minimal::xmlRpcToKindr(T_W_MAP_xml, &T_W_MAP_);
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
  map_parameters.frame_id = map_frame_id_;
  map_parameters.cov_fun = cov_fun;
  map_parameters.log_hyperparams = log_hyperparams;
  map_parameters.T_W_MAP = T_W_MAP_;
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
}

void tmPlannerNode::deleteMarkers() {
  for (size_t i = 0; i < polynomial_markers_.markers.size(); ++i) {
    polynomial_markers_.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  polynomial_pub_.publish(polynomial_markers_);
  path_points_marker_.points.clear();
  path_points_marker_pub_.publish(path_points_marker_);
}

void tmPlannerNode::odometryCallback(
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
  world_frame_id_ = odometry_message->header.frame_id;

  if (!got_odometry_) {
    got_odometry_ = true;
    // Publish world to map transform.
    geometry_msgs::TransformStamped tf_transform_msg;
    geometry_msgs::Transform transform;
    tf_transform_msg.header.stamp = ros::Time::now();
    tf_transform_msg.header.frame_id = world_frame_id_;
    tf_transform_msg.child_frame_id = map_frame_id_;
    tf::transformKindrToMsg(T_W_MAP_, &transform);
    tf_transform_msg.transform = transform;
    static_tf_broadcaster_.sendTransform(tf_transform_msg);
  }
}

void tmPlannerNode::imageCallback(
    const sensor_msgs::ImageConstPtr& image_message) {
  geometry_msgs::Pose odom_pose = odometry_pose_;
  ros::Time odom_time = odom_time_;
  if (abs(image_message->header.stamp.toNSec() - odom_time.toNSec()) >
      0.25 * pow(10, 9.0)) {
    LOG(INFO) << "More than 0.25s delay between image and odometry messages! "
                 "Skipping measurement...";
    return;
  }
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(image_message);
  } catch (cv_bridge::Exception& e) {
    LOG(FATAL) << "cv_bridge exception: ", e.what();
  }
  if (do_map_updates_) {
    tmplanner_.updateMap(cv_ptr, odom_pose);
  }
}

void tmPlannerNode::visualizationTimerCallback(const ros::TimerEvent&) {
  // Polynomial trajectory.
  mav_trajectory_generation::drawMavTrajectory(
      tmplanner_.getTrajectory(), 1.0, world_frame_id_, &polynomial_markers_);
  polynomial_pub_.publish(polynomial_markers_);

  // Control points.
  path_points_marker_.header.frame_id = world_frame_id_;
  path_points_marker_.header.stamp = ros::Time();
  path_points_marker_.ns = "waypoints";
  path_points_marker_.id = 0;
  path_points_marker_.type = visualization_msgs::Marker::CUBE_LIST;
  path_points_marker_.action = visualization_msgs::Marker::ADD;
  path_points_marker_.pose.position.x = 0.0;
  path_points_marker_.pose.position.y = 0.0;
  path_points_marker_.pose.position.z = 0.0;
  path_points_marker_.pose.orientation.x = 0.0;
  path_points_marker_.pose.orientation.y = 0.0;
  path_points_marker_.pose.orientation.z = 0.0;
  path_points_marker_.pose.orientation.w = 1.0;
  path_points_marker_.scale.x = 0.5;
  path_points_marker_.scale.y = 0.5;
  path_points_marker_.scale.z = 0.5;
  path_points_marker_.color.a = 1.0;
  path_points_marker_.color.r = 1.0;
  path_points_marker_.color.g = 0.4;
  path_points_marker_.color.b = 0.4;

  std::deque<geometry_msgs::Pose> control_poses = tmplanner_.getControlPoses();
  path_points_marker_.points.clear();
  for (size_t i = 0; i < control_poses.size(); ++i) {
    if (control_poses.empty()) {
      continue;
    }
    geometry_msgs::Point pt;
    pt.x = control_poses[i].position.x;
    pt.y = control_poses[i].position.y;
    pt.z = control_poses[i].position.z;
    path_points_marker_.points.push_back(pt);
  }
  path_points_marker_pub_.publish(path_points_marker_);
}

void tmPlannerNode::planningTimerCallback(const ros::TimerEvent&) {
  if (tmplanner_.isBudgetSpent()) {
    LOG(INFO) << "Budget spent! Exiting...";
    ros::shutdown();
  }
  double time_along_trajectory =
      ros::Time::now().toSec() - trajectory_publish_time_;

  // Re-plan if the end of the current trajectory has been reached.
  if (time_along_trajectory > tmplanner_.getTrajectory().getMaxTime()) {
    if (!do_map_updates_) {
      do_map_updates_ = true;
    }
    deleteMarkers();
    // Cannot create a new plan: send landing command.
    if (!tmplanner_.createNewPlan(odometry_pose_)) {
      std_srvs::Empty::Request empty_request;
      std_srvs::Empty::Response empty_response;
      landCallback(empty_request, empty_response);
    }
    publishTrajectory();
  }
}

bool tmPlannerNode::startPlanningCallback(std_srvs::Empty::Request& request,
                                          std_srvs::Empty::Response& response) {
  CHECK(got_odometry_) << "No odometry received yet, can't start IPP!";
  // Command the MAV to go to the specified initial position.
  // Cannot create a plan: send landing command.
  tmplanner_.createInitialPlan();
  publishTrajectory();
  visualization_timer_ = nh_.createTimer(
      ros::Duration(0.1), &tmPlannerNode::visualizationTimerCallback, this);
  planning_timer_ = nh_.createTimer(
      ros::Duration(0.1), &tmPlannerNode::planningTimerCallback, this);
  return true;
}

bool tmPlannerNode::landCallback(std_srvs::Empty::Request& request,
                                 std_srvs::Empty::Response& response) {
  LOG(INFO) << "Landing now...";
  do_map_updates_ = false;
  tmplanner_.createLandingPlan();
  planning_timer_.stop();
  publishTrajectory();
  return true;
}

void tmPlannerNode::publishTrajectory() {
  CHECK_GT(tmplanner_.getTrajectory().K(), 0)
      << "Trajectory to publish is empty!";
  planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
      tmplanner_.getTrajectory(), &msg);
  path_segments_pub_.publish(msg);
  trajectory_publish_time_ = ros::Time::now().toSec();
}

int main(int argc, char** argv) {
  // Start the logging.
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  // Initialize ROS, start node.
  ros::init(argc, argv, "tmplanner_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  tmPlannerNode ipp_node(nh, nh_private);
  // Receive callbacks in queue.
  ros::spin();
  return 0;
}