#include <eigen-checks/glog.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <tf/tf.h>

#include "tmplanner_discrete/grid_map.h"

TEST(MapUpdateTest, PredictsUpdate) {
  // Grid map parameters.
  const double width = 30.0;
  const double height = 30.0;
  const double resolution_x = 0.75;
  const double resolution_y = 0.75;
  const double upper_threshold = 0.75;
  const double lower_threshold = 0.25;

  // Sensor parameters.
  const double fov_angle_x = 45.0;
  const double fov_angle_y = 60.0;
  const double saturation_height = 20.0;
  std::vector<double> true_positive_coeffs = {0.00001470588235, -0.01288235294,
                                              0.88286764711};
  std::vector<double> false_negative_coeffs = {-0.0002696078431, 0.02117647059,
                                               0.02909313725};

  // Set the MAV-CAM frame transform.
  Eigen::Matrix4d T;
  T << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 1.0;
  kindr::minimal::QuatTransformation T_IMU_CAM(T);

  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, upper_threshold,
                          lower_threshold, "map");
  grid_map.fillUnknown();
  LOG(INFO) << "Initial entropy = " << grid_map.computeEntropy();

  // Set up candidate MAV pose.
  geometry_msgs::Pose mav_pose;
  mav_pose.position.x = 0.0;
  mav_pose.position.y = 0.0;
  mav_pose.position.z = 10.0;
  mav_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // Predict the map update.
  grid_map.predictMapUpdate(mav_pose, T_IMU_CAM, fov_angle_x, fov_angle_y,
                            saturation_height, true_positive_coeffs,
                            false_negative_coeffs);
  LOG(INFO) << "Final entropy = " << grid_map.computeEntropy();
}

// gtest main
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}