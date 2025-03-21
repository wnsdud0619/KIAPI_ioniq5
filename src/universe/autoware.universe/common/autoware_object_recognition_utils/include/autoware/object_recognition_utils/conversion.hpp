// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_
#define AUTOWARE__OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

namespace autoware::object_recognition_utils
{
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

DetectedObject toDetectedObject(const TrackedObject & tracked_object);
DetectedObjects toDetectedObjects(const TrackedObjects & tracked_objects);
TrackedObject toTrackedObject(const DetectedObject & detected_object);
}  // namespace autoware::object_recognition_utils

#endif  // AUTOWARE__OBJECT_RECOGNITION_UTILS__CONVERSION_HPP_
