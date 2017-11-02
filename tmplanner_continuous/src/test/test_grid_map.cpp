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

#include <eigen-checks/glog.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <tf/tf.h>

#include "grid_map.h"

TEST(GridMapTest, SetsData) {
  // Grid map parameters.
  const double width = 3.0;
  const double height = 3.0;
  const double resolution_x = 0.75;
  const double resolution_y = 0.75;

  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, "map");
  grid_map.fillUnknown();

  Eigen::Matrix4d grid_map_data_ref;
  grid_map_data_ref << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
      0.5, 0.5, 0.5, 0.5, 0.5;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(grid_map.getData(), grid_map_data_ref, 0.01));
}

TEST(GridMapTest, ConvertsCoordinates) {
  // Grid map parameters.
  const double width = 30.0;
  const double height = 30.0;
  const double resolution_x = 0.75;
  const double resolution_y = 0.75;

  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, "map");

  geometry_msgs::Point point;
  // Grid -> environment co-ordinates.
  point.x = 39;
  point.y = 39;
  point.z = 2;
  grid_map.gridToEnvironmentCoordinates(&point);
  EXPECT_NEAR(point.x, 14.25 + 0.375, 0.01);
  EXPECT_NEAR(point.y, 14.25 + 0.375, 0.01);
  point.x = 20;
  point.y = 20;
  point.z = 2;
  grid_map.gridToEnvironmentCoordinates(&point);
  EXPECT_NEAR(point.x, 0.375, 0.01);
  EXPECT_NEAR(point.y, 0.375, 0.01);
  point.x = 10;
  point.y = 10;
  grid_map.gridToEnvironmentCoordinates(&point);
  EXPECT_NEAR(point.x, -7.5 + 0.375, 0.01);
  EXPECT_NEAR(point.y, -7.5 + 0.375, 0.01);
  point.x = 0;
  point.y = 0;
  grid_map.gridToEnvironmentCoordinates(&point);
  EXPECT_NEAR(point.x, -15.0 + 0.375, 0.01);
  EXPECT_NEAR(point.y, -15.0 + 0.375, 0.01);

  // Environment -> grid co-ordinates.
  point.x = 14.25;
  point.y = 14.25;
  grid_map.environmentToGridCoordinates(&point);
  EXPECT_NEAR(point.x, 39, 0.1);
  EXPECT_NEAR(point.y, 39, 0.1);
  point.x = 14.24;
  point.y = 14.24;
  grid_map.environmentToGridCoordinates(&point);
  EXPECT_NEAR(point.x, 38, 0.1);
  EXPECT_NEAR(point.y, 38, 0.1);
  point.x = 14.75;
  point.y = 14;
  grid_map.environmentToGridCoordinates(&point);
  EXPECT_NEAR(point.x, 39, 0.1);
  EXPECT_NEAR(point.y, 38, 0.1);
  point.x = -15;
  point.y = 0.5;
  grid_map.environmentToGridCoordinates(&point);
  EXPECT_NEAR(point.x, 0, 0.1);
  EXPECT_NEAR(point.y, 20, 0.1);
}

TEST(GridMapTest, UpdatesMap) {
  // Grid map parameters
  const double width = 30.0;
  const double height = 30.0;
  const double resolution_x = 0.75;
  const double resolution_y = 0.75;
  // Sensor parameters.
  const double sensor_fov_angle_x = 60.0;
  const double sensor_fov_angle_y = 60.0;

  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, "map");
  grid_map.fillUnknown();

  sensor_msgs::Image image;
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 5.0;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // grid_map.updateMapFromImage(image, pose, sensor_fov_angle_x,
  //                            sensor_fov_angle_y);
}

TEST(GridMapTest, UsesThreshold) {
  // Grid map parameters
  const double width = 30.0;
  const double height = 30.0;
  const double resolution_x = 0.75;
  const double resolution_y = 0.75;
  const std::string cov_fun = "matern3";
  const std::vector<double> log_hyperparams = {1.3, 0.3};
  // Threshold parameter
  const double lower_threshold = 0.5;

  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, "map");
  // Set up the map data, setting some values above the threshold.
  Eigen::MatrixXd grid_map_data;
  grid_map_data = Eigen::MatrixXd::Ones(40, 40) * 0.4;
  grid_map_data(1, 8) = 0.6;
  grid_map_data(21, 32) = 0.6;
  grid_map_data(18, 18) = 0.6;
  grid_map_data(11, 0) = 0.6;
  grid_map.fillMapData(grid_map_data);
  grid_map.computeCovariance(cov_fun, log_hyperparams, resolution_x,
                             resolution_y);

  // Compute the information.
  Eigen::MatrixXd P_diag = grid_map.getCovariance().diagonal();
  Eigen::Map<Eigen::MatrixXd> P(P_diag.data(), grid_map.getData().rows(),
                                grid_map.getData().cols());
  double cov_trace = 0.0;
  for (size_t j = 0; j < grid_map.getData().cols(); ++j) {
    for (size_t i = 0; i < grid_map.getData().rows(); ++i) {
      if (grid_map.getData()(i, j) > lower_threshold) {
        // LOG(INFO) << i << ", " << j;
        // LOG(INFO) << P(i, j);
        cov_trace += P(i, j);
      }
    }
  }
  LOG(INFO) << cov_trace;

  // Print the covariance.
  std::ofstream f;
  f.open(ros::package::getPath("flourish_ipp") + "/debug/P.csv");
  f << grid_map.getCovariance().format(csv_format) << '\n';
}

// TEST(GridMapTest, ProjectsImageToGround) {
//   // Grid map parameters
//   const double width = 30.0;
//   const double height = 30.0;
//   const double resolution_x = 0.75;
//   const double resolution_y = 0.75;
//   const std::string cov_fun = "matern3";
//   const std::vector<double> log_hyperparams = {1.3, 0.3};

//   // Set up the map.
//   grid_map::GridMap grid_map;
//   grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
//                           -width / 2.0, -height / 2.0, "map");

//   // Set the MAV-CAM frame transform.
//   Eigen::Matrix4d T;
//   T << 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
//       0.0, 1.0;
//   kindr::minimal::QuatTransformation T_IMU_CAM(T);

//   // Set the camera intrinsic calibration matrix.
//   const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 554.25, 0, 320.0,
//                                  0.0, 579.41, 240.0, 0.0, 0.0, 1.0);

//   // Load the input test image from file.
//   cv::Mat input_image;
//   input_image = cv::imread(
//       ros::package::getPath("flourish_ipp") + "/debug/images/image1.jpg", 1);

//   // Set the pose of the MAV when the image was taken.
//   geometry_msgs::Pose mav_pose;
//   // image1.jpg.
//   mav_pose.position.x = -9.71249;
//   mav_pose.position.y = 0.00626685;
//   mav_pose.position.z = 13.7127;
//   mav_pose.orientation.x = -0.215215;
//   mav_pose.orientation.y = 0.0228083;
//   mav_pose.orientation.z = -0.00538714;
//   mav_pose.orientation.w = 0.976285;
//   // Run the test.
//   cv::Mat output_image;
//   grid_map.projectImageToGround(input_image, camera_matrix, mav_pose, T_IMU_CAM,
//                                 &output_image);
//   cv::imwrite(ros::package::getPath("flourish_ipp") + "/debug/output_image.jpg",
//               output_image);
// }

// gtest main
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}