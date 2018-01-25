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
#include <gtest/gtest.h>
#include <random>

#include "tmplanner_continuous/cov_fun.h"
#include "tmplanner_continuous/cov_matern3_iso.h"

using namespace gp;

Eigen::IOFormat matlab_format(Eigen::FullPrecision, 0, ", ", ";\n", "", "", "[",
                              "]");

TEST(CovFunTest, ComputesDist) {
  //LOG(INFO) << "Starting test 'ComputesDist'... ";

  typedef double Scalar;
  const int InputDim = 1;
  const size_t NumInputsX = 5;
  const size_t NumInputsY = 4;
  typedef CovMatern3iso<InputDim, Scalar> Cov;

  // Test 1: One input, single-input distance function.
  // Define the inputs.
  Cov::Inputs X(InputDim, NumInputsX);
  X.row(0) =
      Eigen::Matrix<double, 1, Eigen::Dynamic>::LinSpaced(NumInputsX, 1.0, 5.0);
  // LOG(INFO) << X;

  // Define the outputs (distance matrix).
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> D;
  D.resize(NumInputsX, NumInputsX);

  // Calculate distance.
  Cov cov_fun;
  double ell = 1.3;
  double sf2 = 0.3;
  Eigen::Vector2d log_hyperparams;
  log_hyperparams << ell, sf2;
  cov_fun.setLogHyperparams(log_hyperparams);
  cov_fun.sqDistMatrix(X, &D);
  // LOG(INFO) << D;

  // Define the expected outputs (distance matrix).
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> D_ref;
  D_ref.resize(NumInputsX, NumInputsX);
  D_ref << 0, 1, 4, 9, 16, 1, 0, 1, 4, 9, 4, 1, 0, 1, 4, 9, 4, 1, 0, 1, 16, 9,
      4, 1, 0;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(D, D_ref, 0.01));

  // Test 2: One input, double-input distance function.
  cov_fun.sqDistMatrix(X, X, &D);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(D, D_ref, 0.01));

  // Test 3: Two inputs, double-input distance function.
  // Define the inputs.
  Cov::Inputs Y(InputDim, NumInputsY);
  Y.row(0) =
      Eigen::Matrix<double, 1, Eigen::Dynamic>::LinSpaced(NumInputsY, 1.0, 4.0);

  // Define the outputs (distance matrix).
  D.resize(NumInputsX, NumInputsY);

  // Calculate distance.
  cov_fun.sqDistMatrix(X, Y, &D);

  // Define the expected outputs (distance matrix).
  D_ref.resize(NumInputsX, NumInputsY);
  D_ref << 0, 1, 4, 9, 1, 0, 1, 4, 4, 1, 0, 1, 9, 4, 1, 0, 16, 9, 4, 1;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(D, D_ref, 0.01));

  //LOG(INFO) << "Ending test 'ComputesDist'... ";
}

TEST(CovFunTest, ComputesCov) {
  //LOG(INFO) << "Starting test 'ComputesCov'... ";

  typedef double Scalar;
  const int InputDim = 1;
  const size_t NumInputsX = 5;
  const size_t NumInputsY = 4;
  typedef CovMatern3iso<InputDim, Scalar> Cov;

  // Test 1: One input.
  // Define the inputs.
  Cov::Inputs X(InputDim, NumInputsX);
  // generateRandomInputData(X);
  X.row(0) =
      Eigen::Matrix<double, 1, Eigen::Dynamic>::LinSpaced(NumInputsX, 1.0, 5.0);
  // LOG(INFO) << X;

  // Define the outputs (covariance matrix).
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C;
  C.resize(NumInputsX, NumInputsX);

  // Calculate covariance.
  Cov cov_fun;
  double ell = 1.3;
  double sf2 = 0.3;
  Eigen::Vector2d log_hyperparams;
  log_hyperparams << ell, sf2;
  cov_fun.setLogHyperparams(log_hyperparams);
  cov_fun.getCovarianceMatrix(X, &C);
  // LOG(INFO) << C;

  // Define the expected outputs (covariance matrix).
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> C_ref;
  C_ref.resize(NumInputsX, NumInputsX);
  C_ref << 1.8221, 1.673, 1.3781, 1.0683, 0.79649, 1.673, 1.8221, 1.673, 1.3781,
      1.0683, 1.3781, 1.673, 1.8221, 1.673, 1.3781, 1.0683, 1.3781, 1.673,
      1.8221, 1.673, 0.79649, 1.0683, 1.3781, 1.673, 1.8221;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(C, C_ref, 0.01));

  // Test 2: Two inputs.
  // Define the inputs.
  Cov::Inputs Y(InputDim, NumInputsY);
  Y.row(0) =
      Eigen::Matrix<double, 1, Eigen::Dynamic>::LinSpaced(NumInputsY, 1.0, 4.0);

  // Define the outputs (covariance matrix).
  C.resize(NumInputsX, NumInputsY);

  // Calculate covariance.
  cov_fun.getCovarianceMatrix(X, Y, &C);
  //LOG(INFO) << C;

  // Define the expected outputs (covariance matrix).
  C_ref.resize(NumInputsX, NumInputsY);
  C_ref << 1.8221, 1.673, 1.378, 1.068, 1.673, 1.8221, 1.673, 1.378, 1.378,
      1.673, 1.822, 1.673, 1.068, 1.378, 1.673, 1.822, 0.7965, 1.068, 1.378,
      1.673;

  //LOG(INFO) << "Ending test 'ComputesCov'... ";
}
// Creates random input data for the covariance function (0-1 values range)
// void generateRandomInputData(const Inputs& X) {
//   srand(123);
//   for (int i = 0; i < X.rows(); ++i) {
//     for (int j = 0; j < X.cols(); ++j) {
//       X(i, j) = rand();
//     }
//   }
// }

// gtest main
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}