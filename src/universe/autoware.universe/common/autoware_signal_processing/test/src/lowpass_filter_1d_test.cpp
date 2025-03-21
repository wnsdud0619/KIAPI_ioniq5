// Copyright 2021 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/signal_processing/lowpass_filter_1d.hpp"

#include <boost/optional/optional_io.hpp>

#include <gtest/gtest.h>

constexpr double epsilon = 1e-6;

using autoware::signal_processing::LowpassFilter1d;

TEST(lowpass_filter_1d, filter)
{
  LowpassFilter1d lowpass_filter_1d(0.1);

  // initial state
  EXPECT_FALSE(lowpass_filter_1d.getValue());

  // random filter
  EXPECT_NEAR(lowpass_filter_1d.filter(0.0), 0.0, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.filter(1.0), 0.9, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.filter(2.0), 1.89, epsilon);
  EXPECT_NEAR(*lowpass_filter_1d.getValue(), 1.89, epsilon);

  // reset without value
  lowpass_filter_1d.reset();
  EXPECT_FALSE(lowpass_filter_1d.getValue());

  // reset with value
  lowpass_filter_1d.reset(-1.1);
  EXPECT_NEAR(*lowpass_filter_1d.getValue(), -1.1, epsilon);
  EXPECT_NEAR(lowpass_filter_1d.filter(0.0), -0.11, epsilon);
  EXPECT_NEAR(*lowpass_filter_1d.getValue(), -0.11, epsilon);
}
