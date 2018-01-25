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
Scalar COV_FUN_CLASS_TEMPLATE::sqDist(const Input& a, const Input& b) const {
  return (a - b).squaredNorm();
}

COV_FUN_TEMPLATE
void COV_FUN_CLASS_TEMPLATE::sqDistMatrix(
    const Inputs& X,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* D) const {
  CHECK_EQ(D->cols(), X.cols());
  CHECK_EQ(D->rows(), X.cols());

  for (int i = 0; i < X.cols(); i++) {
    for (int j = 0; j <= i; j++) {
      (*D)(i, j) = (*D)(j, i) = sqDist(X.col(i), X.col(j));
    }
  }
}

COV_FUN_TEMPLATE
void COV_FUN_CLASS_TEMPLATE::sqDistMatrix(
    const Inputs& X, const Inputs& Y,
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>* D) const {
  CHECK_EQ(D->rows(), X.cols());
  CHECK_EQ(D->cols(), Y.cols());

  for (int i = 0; i < X.cols(); i++) {
    for (int j = 0; j < Y.cols(); j++) {
      (*D)(i, j) = sqDist(X.col(i), Y.col(j));
    }
  }
}

}  // namespace gp