// Copyright [2025] [Robert Vaughan]
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

#ifndef __CONVERT_H__
#define __CONVERT_H__

#include "cv_detector_ros2_bridge/msg/detections_list.hpp"
#include "detections_list_generated.h"

namespace convert {

/**
 * Convert a flatbuffers detection list to a ROS detection list.
 * @param input flatbuffers detections list
 * @param output ROS detections list
 * @return void
 */
void detections_list(
    const gst_opencv_detector::DetectionList* input,
    cv_detector_ros2_bridge::msg::DetectionsList& output);

}

#endif // __CONVERT_H__