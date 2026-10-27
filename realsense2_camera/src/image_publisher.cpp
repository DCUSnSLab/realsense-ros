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

#include <image_publisher.h>

#include <cv_bridge/cv_bridge.hpp>
#include <cstring>

using namespace realsense2_camera;

namespace
{
size_t image_bytes(const cv::Mat& image)
{
    return image.total() * image.elemSize();
}

void copy_image(const cv::Mat& image, flux::wire::Span<uint8_t> output)
{
    const size_t row_bytes = image.cols * image.elemSize();
    if (output.size() != image_bytes(image))
        return;
    for (int row = 0; row < image.rows; ++row)
        std::memcpy(output.data() + row * row_bytes, image.ptr(row), row_bytes);
}

sensor_msgs::msg::Image::UniquePtr ros_image(
    const cv::Mat& image, const std::string& encoding, const std_msgs::msg::Header& header)
{
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    cv_bridge::CvImage(header, encoding, image).toImageMsg(*message);
    message->is_bigendian = false;
    return message;
}
}

image_flux_publisher::image_flux_publisher(
    RosNodeBase& node, const std::string& topic_name, const flux_channel_config& config) :
    _publisher(node, topic_name, config)
{}

void image_flux_publisher::publish(
    const cv::Mat& image, const std::string& encoding, const std_msgs::msg::Header& header)
{
    auto builder = _publisher.build();
    if (!builder)
        return;
    builder.set__header__stamp(header.stamp.sec, header.stamp.nanosec);
    builder.set__header__frame_id(header.frame_id);
    builder.set__height(image.rows);
    builder.set__width(image.cols);
    builder.set__encoding(encoding);
    builder.set__is_bigendian(false);
    builder.set__step(image.cols * image.elemSize());
    copy_image(image, builder.alloc__data(image_bytes(image)));
    _publisher.publish(builder);
}

size_t image_flux_publisher::get_subscription_count() const
{
    // Flux has no cheap subscriber count; keep the upstream frame-production gate open.
    return 1;
}

image_rcl_publisher::image_rcl_publisher(RosNodeBase& node, const std::string& topic_name,
                                        const rmw_qos_profile_t& qos)
{
    image_publisher_impl = node.create_publisher<sensor_msgs::msg::Image>(
        topic_name, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos), qos));
}

void image_rcl_publisher::publish(
    const cv::Mat& image, const std::string& encoding, const std_msgs::msg::Header& header)
{
    image_publisher_impl->publish(ros_image(image, encoding, header));
}

size_t image_rcl_publisher::get_subscription_count() const
{
    return image_publisher_impl->get_subscription_count();
}

image_transport_publisher::image_transport_publisher(rclcpp::Node& node, const std::string& topic_name,
                                                    const rmw_qos_profile_t& qos)
{
    image_publisher_impl = std::make_shared<image_transport::Publisher>(
        image_transport::create_publisher(&node, topic_name, qos));
}

void image_transport_publisher::publish(
    const cv::Mat& image, const std::string& encoding, const std_msgs::msg::Header& header)
{
    image_publisher_impl->publish(*ros_image(image, encoding, header));
}

size_t image_transport_publisher::get_subscription_count() const
{
    return image_publisher_impl->getNumSubscribers();
}

rgbd_flux_publisher::rgbd_flux_publisher(
    RosNodeBase& node, const std::string& topic_name, const flux_channel_config& config) :
    _publisher(node, topic_name, config)
{}

void rgbd_flux_publisher::publish(
    const cv::Mat& rgb, const cv::Mat& depth,
    const std::string& rgb_encoding, const std::string& depth_encoding,
    const sensor_msgs::msg::CameraInfo& rgb_info, const sensor_msgs::msg::CameraInfo& depth_info,
    const std_msgs::msg::Header& header)
{
    auto builder = _publisher.build();
    if (!builder)
        return;
    builder.set__header__stamp(header.stamp.sec, header.stamp.nanosec);
    builder.set__header__frame_id(header.frame_id);
    builder.set__rgb_camera_info_header_stamp_sec(header.stamp.sec);
    builder.set__rgb_camera_info_header_stamp_nanosec(header.stamp.nanosec);
    builder.set__rgb_camera_info_header_frame_id(rgb_info.header.frame_id);
    builder.set__rgb_camera_info_height(rgb_info.height);
    builder.set__rgb_camera_info_width(rgb_info.width);
    builder.set__rgb_camera_info_distortion_model(rgb_info.distortion_model);
    builder.set__rgb_camera_info_binning_x(rgb_info.binning_x);
    builder.set__rgb_camera_info_binning_y(rgb_info.binning_y);
    builder.set__rgb_camera_info_roi_x_offset(rgb_info.roi.x_offset);
    builder.set__rgb_camera_info_roi_y_offset(rgb_info.roi.y_offset);
    builder.set__rgb_camera_info_roi_height(rgb_info.roi.height);
    builder.set__rgb_camera_info_roi_width(rgb_info.roi.width);
    builder.set__rgb_camera_info_roi_do_rectify(rgb_info.roi.do_rectify);
    auto rgb_camera_info_d = builder.alloc__rgb_camera_info_d(rgb_info.d.size());
    std::copy_n(rgb_info.d.begin(), rgb_camera_info_d.size(), rgb_camera_info_d.begin());
    std::copy(rgb_info.k.begin(), rgb_info.k.end(), builder.rgb_camera_info_k().begin());
    std::copy(rgb_info.r.begin(), rgb_info.r.end(), builder.rgb_camera_info_r().begin());
    std::copy(rgb_info.p.begin(), rgb_info.p.end(), builder.rgb_camera_info_p().begin());
    builder.set__depth_camera_info_header_stamp_sec(header.stamp.sec);
    builder.set__depth_camera_info_header_stamp_nanosec(header.stamp.nanosec);
    builder.set__depth_camera_info_header_frame_id(depth_info.header.frame_id);
    builder.set__depth_camera_info_height(depth_info.height);
    builder.set__depth_camera_info_width(depth_info.width);
    builder.set__depth_camera_info_distortion_model(depth_info.distortion_model);
    builder.set__depth_camera_info_binning_x(depth_info.binning_x);
    builder.set__depth_camera_info_binning_y(depth_info.binning_y);
    builder.set__depth_camera_info_roi_x_offset(depth_info.roi.x_offset);
    builder.set__depth_camera_info_roi_y_offset(depth_info.roi.y_offset);
    builder.set__depth_camera_info_roi_height(depth_info.roi.height);
    builder.set__depth_camera_info_roi_width(depth_info.roi.width);
    builder.set__depth_camera_info_roi_do_rectify(depth_info.roi.do_rectify);
    auto depth_camera_info_d = builder.alloc__depth_camera_info_d(depth_info.d.size());
    std::copy_n(depth_info.d.begin(), depth_camera_info_d.size(), depth_camera_info_d.begin());
    std::copy(depth_info.k.begin(), depth_info.k.end(), builder.depth_camera_info_k().begin());
    std::copy(depth_info.r.begin(), depth_info.r.end(), builder.depth_camera_info_r().begin());
    std::copy(depth_info.p.begin(), depth_info.p.end(), builder.depth_camera_info_p().begin());
    builder.set__rgb_header_stamp_sec(header.stamp.sec);
    builder.set__rgb_header_stamp_nanosec(header.stamp.nanosec);
    builder.set__rgb_header_frame_id(rgb_info.header.frame_id);
    builder.set__rgb_height(rgb.rows);
    builder.set__rgb_width(rgb.cols);
    builder.set__rgb_encoding(rgb_encoding);
    builder.set__rgb_is_bigendian(false);
    builder.set__rgb_step(rgb.cols * rgb.elemSize());
    copy_image(rgb, builder.alloc__rgb_data(image_bytes(rgb)));
    builder.set__depth_header_stamp_sec(header.stamp.sec);
    builder.set__depth_header_stamp_nanosec(header.stamp.nanosec);
    builder.set__depth_header_frame_id(depth_info.header.frame_id);
    builder.set__depth_height(depth.rows);
    builder.set__depth_width(depth.cols);
    builder.set__depth_encoding(depth_encoding);
    builder.set__depth_is_bigendian(false);
    builder.set__depth_step(depth.cols * depth.elemSize());
    copy_image(depth, builder.alloc__depth_data(image_bytes(depth)));
    _publisher.publish(builder);
}

std::shared_ptr<image_publisher> realsense2_camera::make_image_publisher(
    RosNodeBase& node, const std::string& topic_name, const rmw_qos_profile_t& qos,
    const flux_config& flux, bool use_intra_process)
{
    if (flux.use_flux)
        return std::make_shared<image_flux_publisher>(node, topic_name, flux.image);
#ifdef USE_LIFECYCLE_NODE
    (void)use_intra_process;
    return std::make_shared<image_rcl_publisher>(node, topic_name, qos);
#else
    if (use_intra_process)
        return std::make_shared<image_rcl_publisher>(node, topic_name, qos);
    return std::make_shared<image_transport_publisher>(node, topic_name, qos);
#endif
}
