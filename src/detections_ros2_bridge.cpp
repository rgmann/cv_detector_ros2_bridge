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

#include <chrono>
#include <memory>
#include <thread>
#include <boost/asio.hpp>

#include "flatbuffers/flatbuffers.h"

#include "rclcpp/rclcpp.hpp"
#include "cv_detector_ros2_bridge/msg/detections_list.hpp"
#include "detections_list_generated.h"
#include "detections_list_client.hpp"

#include "sync_queue.hpp"
#include "message.hpp"
#include "convert.hpp"

using namespace cv_detector_ros2_bridge;
using namespace std::chrono_literals;

class DetectionBridge : public rclcpp::Node {
public:

    DetectionBridge()
        : Node("cv_detector_ros2_bridge")
    {
        publisher_ = this->create_publisher<cv_detector_ros2_bridge::msg::DetectionsList>(
            "detections_list", 10);
    }

    void publish_detection_list(const msg::DetectionsList& message)
    {
        publisher_->publish(message);
    }


private:

    rclcpp::Publisher<cv_detector_ros2_bridge::msg::DetectionsList>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    boost::asio::io_context io_context;

    sync_queue<message::ptr> queue;

    boost::asio::ip::tcp::resolver resolver(io_context);
    auto endpoints = resolver.resolve(argv[1], argv[2]);

    detections_list_client client(io_context, endpoints, queue);

    std::thread t([&io_context](){ io_context.run(); });

    auto bridge_node = std::make_shared<DetectionBridge>();

    while (true)
    {
        // Wait for a message to become available.
        message::ptr msg;
        if (queue.pop(msg, std::chrono::milliseconds(1000)))
        {
            // Verify the buffer
            flatbuffers::Verifier verifier(msg->body(), msg->body_length());
            if (verifier.VerifyBuffer<gst_opencv_detector::DetectionList>())
            {
                // Parse and convert the flatbuffer to the ROS2 message.
                const gst_opencv_detector::DetectionList* fb_detection_list =
                    flatbuffers::GetRoot<gst_opencv_detector::DetectionList>(msg->body());

                msg::DetectionsList ros_detections_list;
                convert::detections_list(fb_detection_list, ros_detections_list);

                bridge_node->publish_detection_list(ros_detections_list);
            }
        }
    }

    client.close();
    t.join();

    rclcpp::shutdown();
    return 0;
}
