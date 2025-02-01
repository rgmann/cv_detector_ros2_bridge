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
 * @param class_filter if the detection class is in this string of strings,
 *                     it will be included in the output
 *                     (emptry string disables filtering)
 * @param sort_by_conf If true, the highest confidence detection will be returned first
 * @return void
 */
void detections_list(
    const gst_opencv_detector::DetectionList* input,
    cv_detector_ros2_bridge::msg::DetectionsList& output,
    const std::string& class_filter = "",
    bool sort_by_conf = true);

}

#endif // __CONVERT_H__