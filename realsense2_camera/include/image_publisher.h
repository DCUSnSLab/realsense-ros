// Copyright 2023 Intel Corporation. All Rights Reserved.
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

#pragma once

#include "ros_node_base.h"
#include "flux_publisher.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/core.hpp>
#include <realsense2_camera_msgs/flux/rgbd.hpp>

#include <image_transport/image_transport.hpp>

namespace realsense2_camera {
class image_publisher
{
public:
    virtual void publish(const cv::Mat& image, const std::string& encoding,
                         const std_msgs::msg::Header& header) = 0;
    virtual size_t get_subscription_count() const = 0;
    virtual ~image_publisher() = default;
};

class image_flux_publisher : public image_publisher
{
public:
    image_flux_publisher(RosNodeBase& node, const std::string& topic_name, const flux_channel_config& config);
    void publish(const cv::Mat& image, const std::string& encoding,
                 const std_msgs::msg::Header& header) override;
    size_t get_subscription_count() const override;

private:
    flux_publisher<sensor_msgs::flux_msg::Image> _publisher;
};

// Native RCL implementation of an image publisher (needed for intra-process communication)
class image_rcl_publisher : public image_publisher
{
public:
    image_rcl_publisher( RosNodeBase & node,
                         const std::string & topic_name,
                         const rmw_qos_profile_t & qos );
    void publish(const cv::Mat& image, const std::string& encoding,
                 const std_msgs::msg::Header& header) override;
    size_t get_subscription_count() const override;

private:
    rclcpp::Publisher< sensor_msgs::msg::Image >::SharedPtr image_publisher_impl;
};

// image_transport implementation of an image publisher (adds a compressed image topic)
class image_transport_publisher : public image_publisher
{
public:
    image_transport_publisher( rclcpp::Node & node,
                               const std::string & topic_name,
                               const rmw_qos_profile_t & qos );
    void publish(const cv::Mat& image, const std::string& encoding,
                 const std_msgs::msg::Header& header) override;
    size_t get_subscription_count() const override;

private:
    std::shared_ptr< image_transport::Publisher > image_publisher_impl;
};

class rgbd_flux_publisher
{
public:
    rgbd_flux_publisher(RosNodeBase& node, const std::string& topic_name, const flux_channel_config& config);
    void publish(const cv::Mat& rgb, const cv::Mat& depth,
                 const std::string& rgb_encoding, const std::string& depth_encoding,
                 const sensor_msgs::msg::CameraInfo& rgb_info,
                 const sensor_msgs::msg::CameraInfo& depth_info,
                 const std_msgs::msg::Header& header);

private:
    flux_publisher<realsense2_camera_msgs::flux_msg::RGBD> _publisher;
};

std::shared_ptr<image_publisher> make_image_publisher(
    RosNodeBase& node, const std::string& topic_name, const rmw_qos_profile_t& qos,
    const flux_config& flux, bool use_intra_process);

}  // namespace realsense2_camera
