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
  void writeMapData(const Eigen::MatrixXd& map);
  void writeSubmapData(const Eigen::MatrixXd& submap);

  unsigned int map_file_counter_ = 0;
  unsigned int submap_file_counter_ = 0;
};

struct PlanningLogger {
  void writeMeasurementPose(const geometry_msgs::Pose& measurement_pose);
  void writeControlPoses(const std::deque<geometry_msgs::Pose>& control_poses);
  void writeMapMetrics(const double& entropy, const double& classifiction_rate);
};

void MappingLogger::writeMapData(const Eigen::MatrixXd& map) {
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_discrete") + "/debug/map" +
         std::to_string(map_file_counter_) + ".csv");
  f << map.format(csv_format) << '\n';
  f.close();
  map_file_counter_++;
}

void MappingLogger::writeSubmapData(const Eigen::MatrixXd& submap) {
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_discrete") + "/debug/submap" +
         std::to_string(submap_file_counter_) + ".csv");
  f << submap.format(csv_format) << '\n';
  f.close();
  submap_file_counter_++;
}

void PlanningLogger::writeMeasurementPose(
    const geometry_msgs::Pose& measurement_pose) {
  std::ofstream f1, f2;
  f1.open(ros::package::getPath("tmplanner_discrete") +
              "/debug/measurement_poses.dat",
          std::ofstream::app);
  CHECK(f1.is_open()) << "Error opening file!";
  f1 << "Time: " << ros::Time::now().toNSec() << "\n";
  f1 << measurement_pose << "\n";
  f1.close();
  f2.open(ros::package::getPath("tmplanner_discrete") +
              "/debug/measurement_poses_matlab.dat",
          std::ofstream::app);
  CHECK(f2.is_open()) << "Error opening file!";
  f2 << ros::Time::now().toNSec() << ", " << measurement_pose.position.x << ", "
     << measurement_pose.position.y << ", " << measurement_pose.position.z
     << "\n";
  f2.close();
}

void PlanningLogger::writeControlPoses(
    const std::deque<geometry_msgs::Pose>& control_poses) {
  std::ofstream f;
  f.open(ros::package::getPath("tmplanner_discrete") +
             "/debug/control_poses.dat",
         std::ofstream::app);
  CHECK(f.is_open()) << "Error opening file!";
  for (auto pose : control_poses) {
    f << pose.position.x << ", " << pose.position.y << ", " << pose.position.z
      << "\n";
  }
  f.close();
}

void PlanningLogger::writeMapMetrics(const double& entropy,
                                     const double& classifiction_rate) {
  std::ofstream f;
  f.open(
      ros::package::getPath("tmplanner_discrete") + "/debug/map_metrics.dat",
      std::ofstream::app);
  CHECK(f.is_open()) << "Error opening file!";
  f << ros::Time::now().toNSec() << ", " << entropy << ", "
    << classifiction_rate << "; \n";
  f.close();
}

#endif  // LOGGER_H_