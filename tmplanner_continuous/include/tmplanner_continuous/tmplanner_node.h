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

#ifndef TMPLANNER_NODE_H_
#define TMPLANNER_NODE_H_

#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"
#include "minkindr_conversions/kindr_xml.h"

#include "tmplanner_continuous/tmplanner.h"

class tmPlannerNode {
 public:
  tmPlannerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  tmPlannerNode() = delete;
  tmPlannerNode(const tmPlannerNode&) = delete;
  tmPlannerNode& operator=(const tmPlannerNode&) = delete;
  ~tmPlannerNode() = default;

 private:
  // ROS comms.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber image_sub_;
  ros::Publisher path_segments_pub_;
  ros::Publisher polynomial_pub_;
  ros::Publisher path_points_marker_pub_;
  ros::ServiceServer start_planning_srv_;
  ros::ServiceServer land_srv_;
  // World to map transform broadcaster.
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  // Timer for plan setting and execution.
  ros::Timer planning_timer_;
  // Timer for rviz visualization.
  ros::Timer visualization_timer_;

  // Informative path planner.
  tmplanner::tmPlanner tmplanner_;

  // Transformation from world to map reference frame.
  kindr::minimal::QuatTransformation T_W_MAP_;

  bool got_odometry_;
  geometry_msgs::Pose odometry_pose_;
  std::string world_frame_id_;
  std::string map_frame_id_;

  // Time at which current trajectory was published.
  double trajectory_publish_time_;

  // Whether to perform map updates with the registered images.
  bool do_map_updates_;

  // Polynomial trajectory markers.
  visualization_msgs::MarkerArray polynomial_markers_;
  visualization_msgs::Marker path_points_marker_;

  // Fetches parameters from ROS server.
  void loadParameters();

  // Deletes old polynomial path markers.
  void deleteMarkers();

  void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_message);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_message);
  void visualizationTimerCallback(const ros::TimerEvent&);
  void planningTimerCallback(const ros::TimerEvent&);

  // Starts informative path planning.
  bool startPlanningCallback(std_srvs::Empty::Request& request,
                             std_srvs::Empty::Response& response);

  // Sends the command to land.
  bool landCallback(std_srvs::Empty::Request& request,
                    std_srvs::Empty::Response& response);

  // Trajectory execution controls
  void publishTrajectory();

  ros::Time odom_time_;
};

#endif  // TMPLANNER_NODE_H_