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

#include "convert.hpp"

void convert::detections_list(
    const gst_opencv_detector::DetectionList* input,
    cv_detector_ros2_bridge::msg::DetectionsList& output
) {
    output.info.timestamp = input->info()->timestamp();
    output.info.image_width = input->info()->image_width();
    output.info.image_height = input->info()->image_height();
    output.info.crop_width = input->info()->crop_width();
    output.info.crop_height = input->info()->crop_height();
    output.info.elapsed_time_ms = input->info()->elapsed_time_ms();

    auto detections = input->detections();
    for (flatbuffers::uoffset_t index = 0; index < detections->size(); ++index)
    {
        auto input_detection = detections->Get(index);

        auto rect = cv_detector_ros2_bridge::msg::Rect();
        rect.x = input_detection->box()->x();
        rect.y = input_detection->box()->y();
        rect.width = input_detection->box()->width();
        rect.height = input_detection->box()->height();

        auto output_detection = cv_detector_ros2_bridge::msg::Detection();
        output_detection.class_id = input_detection->class_id();
        output_detection.class_name = input_detection->class_name()->str();
        output_detection.box = rect;
        output_detection.confidence = input_detection->confidence();

        output.detections.push_back(output_detection);
    }
}