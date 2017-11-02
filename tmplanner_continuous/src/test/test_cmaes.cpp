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
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <stdio.h>

#include <libcmaes/cmaes.h>

// Full examples from:
// * Unbounded: https://github.com/beniz/libcmaes (README)
// * Bounded: https://github.com/beniz/libcmaes/wiki/Defining-and-using-bounds-on-parameters

using namespace libcmaes;

FitFunc fsphere = [](const double* x, const int N) {
  double val = 0.0;
  for (int i = 0; i < N; i++) val += x[i] * x[i];
  return val;
};

// For some reason, can only run one of these tests at a time??
// Comment out the other one.

// TEST(CmaesTest, OptimizesUnboundedWithCmaes) {
//   LOG(INFO) << "Entered unbounded CMA-ES test.";

//   int dim = 10;  // problem dimensions.
//   std::vector<double> x0(dim, 10.0);
//   double sigma = 0.1;
//   // int lambda = 100; // offsprings at each generation.
//   CMAParameters<> cmaparams(x0, sigma);
//   // cmaparams.set_algo(BIPOP_CMAES);
//   CMASolutions cmasols = cmaes<>(fsphere, cmaparams);
//   std::cout << "best solution: " << cmasols << std::endl;
//   std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0
//             << " seconds\n";
// }

TEST(CmaesTest, OptimizesBoundedWithCmaes) {
  LOG(INFO) << "Entered bounded CMA-ES test.";

  int dim = 10;  // problem dimensions.
  double sigma = 0.1;
  double lbounds[dim], ubounds[dim];  // arrays for lower and upper parameter
                                      // bounds, respectively
  for (int i = 0; i < dim; i++) {
    lbounds[i] = -2.0;
    ubounds[i] = 2.0;
  }

  std::vector<double> x0(dim, 1.0);  // beware that x0 is within bounds.
  GenoPheno<pwqBoundStrategy> geno(
      lbounds, ubounds,
      dim);  // genotype / phenotype transform associated to bounds.
  CMAParameters<GenoPheno<pwqBoundStrategy>> cmaparams(
      dim, &x0.front(), sigma, -1, 0, geno);  // -1 for automatically \
// decided lambda, 0 is for random seeding of the internal generator.
  CMASolutions cmasols = cmaes<GenoPheno<pwqBoundStrategy>>(fsphere, cmaparams);
  std::cout << "best solution: ";
  cmasols.print(std::cout, 0, geno);
  std::cout << std::endl;
  std::cout << "optimization took " << cmasols.elapsed_time() / 1000.0
            << " seconds\n";
}

// gtest main
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}