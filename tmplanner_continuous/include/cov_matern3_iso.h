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

#include "cov_fun.h"

#ifndef COV_MATERN3_ISO_H_
#define COV_MATERN3_ISO_H_

#define COV_MATERN_3_ISO_CLASS_TEMPLATE CovMatern3iso<InputDim, Scalar>

namespace gp {

COV_FUN_TEMPLATE
class CovMatern3iso : public COV_FUN_CLASS_TEMPLATE {
 public:
  typedef Eigen::Matrix<Scalar, InputDim, 1> Input;
  typedef Eigen::Matrix<Scalar, InputDim, Eigen::Dynamic> Inputs;

  CovMatern3iso();
  void setLogHyperparams(const Eigen::VectorXd& log_hyperparams);

  Scalar getCovarianceElement(const Inputs& a, const Inputs& b) const;
  void getCovarianceMatrix(
      const Inputs& X,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* C) const;
  void getCovarianceMatrix(
      const Inputs& X, const Inputs& Y,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* C) const;

  // Gets Matern-3 iso covariance for given seperation(s) between points.
  Scalar computeCovariancefromDist(const Scalar& distance) const;
  void computeCovariancefromDist(
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* D) const;

 private:
  // Length scale.
  double ell_;
  // Signal variance squared.
  double sf2_;
  // Square root of 3 (constant).
  double sqrt3_;
};
} // namespace gp

#include "impl/cov_matern3_iso_impl.h"

#endif  // COV_MATERN3_ISO_H_