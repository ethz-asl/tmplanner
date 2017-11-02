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

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

#ifndef COV_FUN_H_
#define COV_FUN_H_

#define COV_FUN_TEMPLATE template <int InputDim, typename Scalar>
#define COV_FUN_CLASS_TEMPLATE CovFun<InputDim, Scalar>

namespace gp {

COV_FUN_TEMPLATE
class CovFun {
 public:
  typedef Eigen::Matrix<Scalar, InputDim, 1> Input;
  typedef Eigen::Matrix<Scalar, InputDim, Eigen::Dynamic> Inputs;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CovFun() = default;
  CovFun(const CovFun&) = default;
  CovFun& operator=(const CovFun&) = default;
  ~CovFun() = default;

  // Sets the log hyperparameters of the covariance function.
  virtual void setLogHyperparams(const Eigen::VectorXd& log_hyperparams) = 0;

  size_t getParamsDim() { return param_dim_; }
  Eigen::VectorXd getLogHyperparams() { return log_hyperparams_; }

  // Gets the covariance value between two input points.
  virtual Scalar getCovarianceElement(const Inputs& a,
                                      const Inputs& b) const = 0;
  // Gets the covariance matrix between all input point pairs.
  virtual void getCovarianceMatrix(
      const Inputs& X,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* C) const = 0;

  // Computes the square distance between two input vectors.
  Scalar sqDist(const Input& u, const Input& v) const;
  // Computes the matrix of square distances between all input pairs.
  void sqDistMatrix(
      const Inputs& X,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* D) const;
  // Computes the matrix of square distances between given input pairs.
  void sqDistMatrix(
      const Inputs& X, const Inputs& Y,
      Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* D) const;

 protected:
  // Kernel name.
  std::string name_;
  // Size of hyperparameter vector.
  size_t param_dim_;
  // Log hyperparameters of the covariance function.
  Eigen::VectorXd log_hyperparams_;
};
}  // namespace gp

#include "impl/cov_fun_impl.h"

#endif  // COV_FUN_H_