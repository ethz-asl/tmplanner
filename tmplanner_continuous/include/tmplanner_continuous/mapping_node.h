#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>

#include "minkindr_conversions/kindr_msg.h"
#include "minkindr_conversions/kindr_tf.h"
#include "minkindr_conversions/kindr_xml.h"

#include "tmplanner_continuous/tmplanner.h"

#ifndef MAPPING_NODE_H_
#define MAPPING_NODE_H_

class MappingNode {
 public:
  MappingNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  MappingNode() = delete;
  MappingNode(const MappingNode&) = delete;
  MappingNode& operator=(const MappingNode&) = delete;
  ~MappingNode() = default;

 private:
  // ROS comms
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber image_sub_;

  // Informative path planner
  tmplanner::tmPlanner tmplanner_;

  // Transformation from world to map
  kindr::minimal::QuatTransformation T_W_MAP_;

  bool got_odometry_;
  geometry_msgs::Pose odometry_pose_;

  // Fetches parameters from ROS server.
  void loadParameters();

  void odometryCallback(const nav_msgs::OdometryConstPtr& odometry_message);
  void imageCallback(const sensor_msgs::ImageConstPtr& image_message);

  ros::Time odom_time_;
};

#endif  // MAPPING_NODE_H_