#ifndef LOGGER_H_
#define LOGGER_H_

#include <time.h>
#include <deque>
#include <fstream>
#include <iostream>

#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

Eigen::IOFormat matlab_format(Eigen::StreamPrecision, 0, ", ", ";\n", "", "",
                              "[", "]");
Eigen::IOFormat csv_format(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ",
                           "\n");

struct MappingLogger {
  void writeSubmapData(const Eigen::MatrixXd& submap);
  void writeKFUpdateData(const Eigen::VectorXd& z, const Eigen::MatrixXd& H,
                         const double& var, const Eigen::MatrixXd& P_prior,
                         const Eigen::MatrixXd& x_prior);
  void writeKFUpdateData(const Eigen::MatrixXd& H, const double& var,
                         const Eigen::MatrixXd& P_prior,
                         const Eigen::MatrixXd& x_prior);
  void writeMeasurementData(const geometry_msgs::Pose& camera_pose,
                            const Eigen::VectorXd& z,
                            const Eigen::MatrixXd& z_ind);
  void writeImageProjectionData(const cv::Mat& input_image,
                                const geometry_msgs::Pose& mav_pose);

  unsigned int submap_file_counter_ = 0;
  unsigned int kf_file_counter_ = 0;
  unsigned int measurement_file_counter_ = 0;
  unsigned int image_projection_file_counter_ = 0;
};

struct PlanningLogger {
  void writeMeasurementPose(const geometry_msgs::Pose& measurement_pose);
  void writeImage(const cv_bridge::CvImagePtr& cv_image_ptr);
  void writeControlPoses(const std::deque<geometry_msgs::Pose>& control_poses);
  void writeCovarianceTrace(const double& cov_trace);
  unsigned int image_counter_ = 0;
};

void MappingLogger::writeSubmapData(const Eigen::MatrixXd& submap) {
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_continuous") + "/debug/submap" +
         std::to_string(submap_file_counter_) + ".csv");
  f << submap.format(csv_format) << '\n';
  f.close();
  submap_file_counter_++;
}

void MappingLogger::writeMeasurementData(const geometry_msgs::Pose& camera_pose,
                                         const Eigen::VectorXd& z,
                                         const Eigen::MatrixXd& z_ind) {
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_continuous") + "/debug/meas" +
         std::to_string(measurement_file_counter_) + ".txt");
  CHECK(f.is_open()) << "Error opening file!";
  f << "Camera position = \n"
    << camera_pose.position.x << " " << camera_pose.position.y << " "
    << camera_pose.position.z << '\n';
  f << "Camera orientation = \n"
    << camera_pose.orientation.x << " " << camera_pose.orientation.y << " "
    << camera_pose.orientation.z << " " << camera_pose.orientation.w << '\n';
  f << "z = " << z.format(matlab_format) << "; \n";
  f << "z_ind = " << z_ind.format(matlab_format) << "; \n";
  f.close();
  measurement_file_counter_++;
}

void MappingLogger::writeKFUpdateData(const Eigen::VectorXd& z,
                                      const Eigen::MatrixXd& H,
                                      const double& var,
                                      const Eigen::MatrixXd& P_prior,
                                      const Eigen::MatrixXd& x_prior) {
  std::ofstream f1, f2, f3;
  //  f1.open(ros::package::getPath("tmplanner_continuous") + "/debug/kf" +
  //         std::to_string(kf_file_counter_) + ".txt");
  // CHECK(f1.is_open()) << "Error opening file!";
  // f1 << "z = " << z.format(matlab_format) << "; \n";
  // f1 << "H = " << H.format(matlab_format) << "; \n";
  // f1 << "var = " << var << "; \n";
  // f1.close();
  // f2.open(ros::package::getPath("tmplanner_continuous") + "/debug/P" +
  //         std::to_string(kf_file_counter_) + ".csv");
  // CHECK(f2.is_open()) << "Error opening file!";
  // f2 << P_prior.format(csv_format) << '\n';
  // f2.close();
  f3.open(ros::package::getPath("tmplanner_continuous") + "/debug/X" +
          std::to_string(kf_file_counter_) + ".csv");
  CHECK(f3.is_open()) << "Error opening file!";
  f3 << x_prior.format(csv_format) << "\n";
  f3.close();
  kf_file_counter_++;
}

void MappingLogger::writeKFUpdateData(const Eigen::MatrixXd& H,
                                      const double& var,
                                      const Eigen::MatrixXd& P_prior,
                                      const Eigen::MatrixXd& x_prior) {
  std::ofstream f1, f2, f3;
  f1.open(ros::package::getPath("tmplanner_continuous") + "/debug/kf" +
          std::to_string(kf_file_counter_) + ".txt");
  CHECK(f1.is_open()) << "Error opening file!";
  f1 << "H = " << H.format(matlab_format) << "; \n";
  f1 << "var = " << var << "; \n";
  f1.close();
  f2.open(ros::package::getPath("tmplanner_continuous") + "/debug/P" +
          std::to_string(kf_file_counter_) + ".csv");
  CHECK(f2.is_open()) << "Error opening file!";
  f2 << P_prior.format(csv_format) << "\n";
  f2.close();
  // f3.open(ros::package::getPath("tmplanner_continuous") + "/debug/X" +
  //        std::to_string(kf_file_counter_) + ".csv");
  // CHECK(f3.is_open()) << "Error opening file!";
  // f3 << x_prior.format(csv_format) << "\n";
  // f3.close();
  kf_file_counter_++;
}

void MappingLogger::writeImageProjectionData(
    const cv::Mat& input_image, const geometry_msgs::Pose& mav_pose) {
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_continuous") +
         "/debug/image_projection_pose" +
         std::to_string(image_projection_file_counter_) + ".txt");
  f << mav_pose << "\n";
  cv::imwrite(ros::package::getPath("tmplanner_continuous") +
                  "/debug/image_projection" +
                  std::to_string(image_projection_file_counter_) + ".jpg",
              input_image);
  image_projection_file_counter_++;
  f.close();
}

void PlanningLogger::writeMeasurementPose(
    const geometry_msgs::Pose& measurement_pose) {
  std::ofstream f1, f2;
  f1.open(ros::package::getPath("tmplanner_continuous") +
              "/debug/measurement_poses.dat",
          std::ofstream::app);
  CHECK(f1.is_open()) << "Error opening file!";
  f1 << "Time: " << ros::Time::now().toNSec() << "\n";
  f1 << measurement_pose << "\n";
  f1.close();
  f2.open(ros::package::getPath("tmplanner_continuous") +
              "/debug/measurement_poses_matlab.dat",
          std::ofstream::app);
  CHECK(f2.is_open()) << "Error opening file!";
  f2 << ros::Time::now().toNSec() << ", " << measurement_pose.position.x << ", "
     << measurement_pose.position.y << ", " << measurement_pose.position.z
     << "\n";
  f2.close();
}

void PlanningLogger::writeImage(const cv_bridge::CvImagePtr& cv_image_ptr) {
  // cv::Mat image = cv_image_ptr->image;
  // cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
  cv::imwrite(ros::package::getPath("tmplanner_continuous") +
                  "/debug/recorded_images/image" +
                  std::to_string(image_counter_) + ".jpg",
              cv_image_ptr->image);
  image_counter_++;
}

void PlanningLogger::writeControlPoses(
    const std::deque<geometry_msgs::Pose>& control_poses) {
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_continuous") +
             "/debug/control_poses_matlab.dat",
         std::ofstream::app);
  CHECK(f.is_open()) << "Error opening file!";
  for (auto pose : control_poses) {
    f << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
      << "\n";
  }
  f.close();
}

void PlanningLogger::writeCovarianceTrace(const double& trace) {
  std::ofstream f;
  f.open(
      ros::package::getPath("tmplanner_continuous") + "/debug/cov_traces.dat",
      std::ofstream::app);
  CHECK(f.is_open()) << "Error opening file!";
  f << ros::Time::now().toNSec() << ", " << trace << "; \n";
  f.close();
}

#endif  // LOGGER_H_