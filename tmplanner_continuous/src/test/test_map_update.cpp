#include <chrono>

#include <eigen-checks/glog.h>
#include <eigen-checks/gtest.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <tf/tf.h>

#include "cov_fun.h"
#include "cov_matern3_iso.h"
#include "grid_map.h"

TEST(MapUpdateTest, PredictsUpdate) {
  // Grid map parameters.
  const double width = 30.0;
  const double height = 30.0;
  const double resolution_x = 0.75;
  const double resolution_y = 0.75;
  const std::string cov_fun = "matern3";
  const std::vector<double> log_hyperparams = {1.3, 0.3};

  // Sensor parameters.
  const double fov_angle_x = 45.0;
  const double fov_angle_y = 60.0;
  const double coefficient_A = 0.05;
  const double coefficient_B = 0.2;

  // Set the MAV-CAM frame transform.
  Eigen::Matrix4d T;
  T << 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 1.0;
  kindr::minimal::QuatTransformation T_IMU_CAM(T);

  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, "map");
  grid_map.fillUnknown();
  grid_map.computeCovariance(cov_fun, log_hyperparams, resolution_x,
                             resolution_y);

  // Set up candidate MAV pose.
  geometry_msgs::Pose mav_pose;
  mav_pose.position.x = 0.0;
  mav_pose.position.y = 0.0;
  mav_pose.position.z = 25.0;
  mav_pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // Predict the map update.
  grid_map.predictMapUpdate(mav_pose, T_IMU_CAM, fov_angle_x, fov_angle_y,
                            coefficient_A, coefficient_B);

  // Print update result.
  std::ofstream f2;
  f2.open(ros::package::getPath("flourish_ipp_continuous") +
          "/debug/P_post_predict.csv");
  f2 << grid_map.getCovariance().format(csv_format) << '\n';
}

TEST(MapUpdateTest, PerformsUpdateFromImage) {
  // Grid map parameters.
  const double width = 30.0;
  const double height = 30.0;
  const double resolution_x = 0.75;
  const double resolution_y = 0.75;
  const std::string cov_fun = "matern3";
  const std::vector<double> log_hyperparams = {1.3, 0.3};
  // Raven
  // const double width = 8.0;
  // const double height = 8.0;
  // const double resolution_x = 0.2;
  // const double resolution_y = 0.2;
  // const std::string cov_fun = "matern3";
  // const std::vector<double> log_hyperparams = {0.2321, -0.75};
  // Sensor parameters.
  const double fov_angle_x = 45.0;
  const double fov_angle_y = 60.0;
  const double coefficient_A = 0.05;
  const double coefficient_B = 0.2;
  // Raven
  // const double fov_angle_x = 42.5;
  // const double fov_angle_y = 55.0;
  // const double coefficient_A = 0.06;
  // const double coefficient_B = 0.2;

  // Set up the map.
  grid_map::GridMap grid_map;
  grid_map.setMapGeometry(width, height, resolution_x, resolution_y,
                          -width / 2.0, -height / 2.0, "map");
  grid_map.fillUnknown();
  grid_map.computeCovariance(cov_fun, log_hyperparams, resolution_x,
                             resolution_y);

  // Set the MAV-CAM frame transform.
  Eigen::Matrix4d T;
  T << 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 1.0;
  // T << -0.00460918823092003, -0.996548933634229, -0.0828792993256664, 0.0408,
  //     -0.999961664231832, 0.00521023827478786, -0.00703729239167134, 0.1519,
  //     0.00744482712612513, 0.0828436858787765, -0.996534745134088, -0.0621,
  //     0.0, 0.0, 0.0, 1.0;
  kindr::minimal::QuatTransformation T_IMU_CAM(T);

  // Set the camera intrinsic calibration matrix.
  const cv::Mat intrinsics_matrix = (cv::Mat_<double>(3, 3) << 554.25, 0, 320.0,
                                     0.0, 579.41, 240.0, 0.0, 0.0, 1.0);
  // const cv::Mat intrinsics_matrix =
  //     (cv::Mat_<double>(3, 3) << 1842.21, 0, 954.3, 0.0, 1185.94, 527.085,
  //     0.0, 0.0, 1.0);

  // Set the pose of the MAV when the image was taken.
  geometry_msgs::Pose mav_pose;
  // image4.jpg.
  mav_pose.position.x = 9.55992;
  mav_pose.position.y = -0.000475374;
  mav_pose.position.z = 20.0051;
  mav_pose.orientation.x = 0.000165516;
  mav_pose.orientation.y = -0.194331;
  mav_pose.orientation.z = 4.84563e-05;
  mav_pose.orientation.w = 0.980936;
  // mav_pose.position.x = 0;
  // mav_pose.position.y = 0;
  // mav_pose.position.z = 20.0051;
  // mav_pose.orientation.x = 0;
  // mav_pose.orientation.y = 0;
  // mav_pose.orientation.z = 0;
  // mav_pose.orientation.w = 1;

  // Load the input test image from file.
  cv::Mat cv_image =
      cv::imread(ros::package::getPath("flourish_ipp_continuous") +
                     "/debug/images/image_map_update4.jpg",
                 1);
  sensor_msgs::ImagePtr image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
  cv_bridge::CvImageConstPtr cv_image_ptr =
      cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);

  // Perform the map update.
  grid_map.updateMapFromImage(cv_image_ptr, mav_pose, T_IMU_CAM, fov_angle_x,
                              fov_angle_y, intrinsics_matrix, coefficient_A,
                              coefficient_B);

  // Print the posterior mean.
  std::ofstream f;
  f.open(ros::package::getPath("flourish_ipp_continuous") +
         "/debug/X_post_update.csv");
  f << grid_map.getData().format(csv_format) << '\n';
}

// gtest main
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}