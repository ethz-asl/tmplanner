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

namespace gp {

COV_FUN_TEMPLATE
COV_MATERN_3_ISO_CLASS_TEMPLATE::CovMatern3iso() {
  this->name_ = ("Matern_3_iso");
  this->param_dim_ = 2;
  this->log_hyperparams_.resize(this->param_dim_);
  this->log_hyperparams_.setZero();
  this->sqrt3_ = sqrt(3);
}

COV_FUN_TEMPLATE
void COV_MATERN_3_ISO_CLASS_TEMPLATE::setLogHyperparams(
    const Eigen::VectorXd& log_hyperparams) {
  CHECK(log_hyperparams.size() == this->param_dim_);
  CHECK_GT(log_hyperparams(0), 0.0) << "Invalid length scale parameter!";
 // CHECK_GT(log_hyperparams(1), 0.0) << "Invalid variance parameter!";
  ell_ = exp(log_hyperparams(0));
  sf2_ = exp(2 * log_hyperparams(1));
}

COV_FUN_TEMPLATE
Scalar COV_MATERN_3_ISO_CLASS_TEMPLATE::getCovarianceElement(
    const Inputs& a, const Inputs& b) const {
  Scalar distance = this->sqDist(a, b);
  return computeCovariancefromDist(distance);
}

COV_FUN_TEMPLATE
void COV_MATERN_3_ISO_CLASS_TEMPLATE::getCovarianceMatrix(
    const Inputs& X,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* C) const {
  // Compute squared distance matrix.
  this->sqDistMatrix(X, C);
  // Compute covariance matrix for all input pairs.
  computeCovariancefromDist(C);
}

COV_FUN_TEMPLATE
void COV_MATERN_3_ISO_CLASS_TEMPLATE::getCovarianceMatrix(
    const Inputs& X, const Inputs& Y,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* C) const {
  // Compute squared distance matrix.
  this->sqDistMatrix(X, Y, C);
  // Compute covariance matrix for input pairs.
  computeCovariancefromDist(C);
}

COV_FUN_TEMPLATE
Scalar COV_MATERN_3_ISO_CLASS_TEMPLATE::computeCovariancefromDist(
    const Scalar& distance) const {
  Scalar z = pow(distance, 0.5) * sqrt3_ / ell_;
  return sf2_ * exp(-z) * (1 + z);
}

COV_FUN_TEMPLATE
void COV_MATERN_3_ISO_CLASS_TEMPLATE::computeCovariancefromDist(
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* D) const {
  // Distance matrix is symmetric.
  if ((*D).isApprox((*D).transpose())) {
    for (int i = 0; i < D->rows(); i++) {
      for (int j = 0; j <= i; j++) {
        (*D)(i, j) = (*D)(j, i) = computeCovariancefromDist((*D)(i, j));
      }
    }
    // Distance matrix is not symmetric.
  } else {
    for (int i = 0; i < D->rows(); i++) {
      for (int j = 0; j < D->cols(); j++) {
        (*D)(i, j) = computeCovariancefromDist((*D)(i, j));
      }
    }
  }
}
}  // namespace gp