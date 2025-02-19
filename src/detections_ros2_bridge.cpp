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
#include <stdexcept>
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

    static const std::string kParamKeyHost;
    static const std::string kParamKeyPort;
    static const std::string kParamKeyClassFilter;
    static const std::string kParamKeySortByConfidence;

    static const std::string kDefaultHost;
    static const std::string kDefaultPort;

    DetectionBridge() : Node("cv_detector_ros2_bridge")
    {
        this->declare_parameter(kParamKeyHost, kDefaultHost);
        this->declare_parameter(kParamKeyPort, kDefaultPort);
        this->declare_parameter(kParamKeyClassFilter, "");
        this->declare_parameter(kParamKeySortByConfidence, true);

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

const std::string DetectionBridge::kParamKeyHost = "host";
const std::string DetectionBridge::kParamKeyPort = "port";
const std::string DetectionBridge::kParamKeyClassFilter = "class-filter";
const std::string DetectionBridge::kParamKeySortByConfidence = "sort-by-confidence";

const std::string DetectionBridge::kDefaultHost = "127.0.0.1";
const std::string DetectionBridge::kDefaultPort = "5050";


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    sync_queue<message::ptr> queue;
    auto bridge_node = std::make_shared<DetectionBridge>();

    boost::asio::io_context io_context;
    boost::asio::ip::tcp::resolver resolver(io_context);

    std::string host = DetectionBridge::kDefaultHost;
    try
    {
        rclcpp::Parameter host_param = bridge_node->get_parameter(
            DetectionBridge::kParamKeyHost);
        host = host_param.get_value<std::string>();
    }
    catch (const std::runtime_error& error)
    {
        RCLCPP_WARN(bridge_node->get_logger(),
            "Defaulting to %s='%s'",
            DetectionBridge::kParamKeyHost.c_str(),
            host.c_str());
    }

    std::string port = DetectionBridge::kDefaultPort;
    try
    {
        rclcpp::Parameter port_param = bridge_node->get_parameter("port");
        port = port_param.get_value<std::string>();
    }
    catch (const std::runtime_error& error)
    {
        RCLCPP_WARN(bridge_node->get_logger(), 
            "Defaulting to %s='%s'",
            DetectionBridge::kParamKeyPort.c_str(),
            port.c_str());
    }

    std::string class_filter;
    try
    {
        rclcpp::Parameter class_filter_param = bridge_node->get_parameter(
            DetectionBridge::kParamKeyClassFilter);
        class_filter = class_filter_param.get_value<std::string>();
    }
    catch (const std::runtime_error& error)
    {
        RCLCPP_WARN(bridge_node->get_logger(),
            "Defaulting to %s='%s'",
            DetectionBridge::kParamKeyClassFilter.c_str(),
            class_filter.c_str());
    }

    bool sort_by_conf = true;
    try
    {
        rclcpp::Parameter sort_param = bridge_node->get_parameter(
            DetectionBridge::kParamKeySortByConfidence);
        sort_by_conf = sort_param.get_value<bool>();
    }
    catch (const std::runtime_error& error)
    {
        RCLCPP_WARN(bridge_node->get_logger(),
            "Defaulting to %s='%s'",
            DetectionBridge::kParamKeySortByConfidence.c_str(),
            sort_by_conf ? "true" : "false");
    }

    auto endpoints = resolver.resolve(host, port);

    detections_list_client client(io_context, endpoints, queue);

    std::thread t([&io_context](){ io_context.run(); });

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
                convert::detections_list(
                    fb_detection_list,
                    ros_detections_list,
                    class_filter,
                    sort_by_conf
                );

                bridge_node->publish_detection_list(ros_detections_list);
            }
        }

        rclcpp::spin_some(bridge_node);
    }

    client.close();
    t.join();

    rclcpp::shutdown();
    return 0;
}
