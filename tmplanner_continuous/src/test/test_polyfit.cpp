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

#include "tmplanner_tools/matlab_tools.h"

TEST(PolyfitTest, GetsCoeffs) {
  const unsigned int order = 1;
  const unsigned int count_of_elements = 2;
  int result;

  double x_data[count_of_elements] = {1.0, 26.0};
  double y_data[count_of_elements] = {16.0, 1.0};
  double coefficients[order + 1];

  result = polyfit(x_data, y_data, count_of_elements, order, coefficients);

  EXPECT_NEAR(16.6, coefficients[0], 0.01);
  EXPECT_NEAR(-0.6, coefficients[1], 0.01);
}

// gtest main
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = 1;
  return RUN_ALL_TESTS();
}