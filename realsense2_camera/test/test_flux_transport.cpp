#include <gtest/gtest.h>
#include <image_publisher.h>
#include <pointcloud_filter.h>
#include <sensor_msgs/flux/image_ros.hpp>
#include <sensor_msgs/flux/point_cloud2_ros.hpp>
#include <realsense2_camera_msgs/flux/rgbd_ros.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <librealsense2/hpp/rs_internal.hpp>
#include <flux/subscription.hpp>
#include <chrono>
#include <array>
#include <cstring>
#include <thread>
#include <unistd.h>

using namespace realsense2_camera;

namespace
{
rs2::frameset synthetic_frames(rs2_format texture_format)
{
    rs2::software_device device;
    auto sensor = device.add_sensor("test_sensor");
    rs2_intrinsics intrinsics{};
    intrinsics.width = 8;
    intrinsics.height = 2;
    intrinsics.ppx = 3.5f;
    intrinsics.ppy = 0.5f;
    intrinsics.fx = 8;
    intrinsics.fy = 2;
    auto depth_profile = sensor.add_video_stream(
        {RS2_STREAM_DEPTH, 0, 0, 8, 2, 30, 2, RS2_FORMAT_Z16, intrinsics});
    const int texture_bytes = texture_format == RS2_FORMAT_RGB8 ? 3 : 1;
    auto texture_profile = sensor.add_video_stream({
        texture_format == RS2_FORMAT_RGB8 ? RS2_STREAM_COLOR : RS2_STREAM_INFRARED,
        0, 1, 8, 2, 30, texture_bytes, texture_format, intrinsics});
    rs2_extrinsics extrinsics{};
    extrinsics.rotation[0] = extrinsics.rotation[4] = extrinsics.rotation[8] = 1;
    depth_profile.register_extrinsics_to(texture_profile, extrinsics);
    sensor.add_read_only_option(RS2_OPTION_DEPTH_UNITS, 0.001f);
    rs2::frame_queue queue;
    sensor.open({depth_profile, texture_profile});
    sensor.start(queue);
    auto* depths = new uint16_t[16];
    std::fill_n(depths, 16, 1000);
    depths[1] = 0;
    sensor.on_video_frame({depths, [](void* p) { delete[] static_cast<uint16_t*>(p); },
        16, 2, 1, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, 1, depth_profile.get(), 0.001f});
    auto depth = queue.wait_for_frame();
    auto* pixels = new uint8_t[16 * texture_bytes];
    for (int index = 0; index < 16 * texture_bytes; ++index)
        pixels[index] = index + 1;
    sensor.on_video_frame({pixels, [](void* p) { delete[] static_cast<uint8_t*>(p); },
        8 * texture_bytes, texture_bytes, 1, RS2_TIMESTAMP_DOMAIN_HARDWARE_CLOCK, 1, texture_profile.get(), 0});
    auto texture = queue.wait_for_frame();
    sensor.stop();
    sensor.close();
    rs2::processing_block combine([&](rs2::frame, rs2::frame_source& source) {
        source.frame_ready(source.allocate_composite_frame({depth, texture}));
    });
    combine.start(queue);
    combine.invoke(depth);
    return queue.wait_for_frame().as<rs2::frameset>();
}
}

class FluxTransport : public testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        const std::string destination = "/realsense_flux_test_" + std::to_string(getpid()) + "/image";
        node = std::make_shared<rclcpp::Node>(
            "camera", rclcpp::NodeOptions().use_intra_process_comms(true).arguments({"--ros-args", "-r", "~/image:=" + destination}));
    }

    sensor_msgs::msg::Image image()
    {
        sensor_msgs::msg::Image message;
        message.header.stamp.sec = 123;
        message.header.stamp.nanosec = 456;
        message.header.frame_id = "camera_optical_frame";
        message.height = 2;
        message.width = 3;
        message.encoding = "16UC1";
        message.is_bigendian = 0;
        message.step = 6;
        message.data = {0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 6};
        return message;
    }

    std::shared_ptr<rclcpp::Node> node;
};

TEST_F(FluxTransport, FluxImagePreservesPixelsMetadataAndRemapping)
{
    auto publisher = make_image_publisher(*node, "~/image", rmw_qos_profile_default, true, false);
    flux::Subscription subscriber(*node, "~/image", sensor_msgs::flux_msg::Image::kFingerprint);
    const auto expected = image();
    publisher->publish(cv::Mat(expected.height, expected.width, CV_16UC1,
                const_cast<uint8_t*>(expected.data.data())), expected.encoding, expected.header);
    auto frame = subscriber.take_blocking(1000000000);
    ASSERT_TRUE(frame);
    const auto received = sensor_msgs::flux_msg::frame_to_msg(sensor_msgs::flux_msg::Image::View(frame));
    EXPECT_EQ(received, expected);
    EXPECT_EQ(node->count_publishers(subscriber.topic_name()), 0U);
    EXPECT_EQ(subscriber.topic_name(), "/realsense_flux_test_" + std::to_string(getpid()) + "/image");
}

TEST_F(FluxTransport, RosImageRetainsNativeAndImageTransportPaths)
{
    for (bool intra_process : {false, true})
    {
        auto publisher = make_image_publisher(*node, "~/image", rmw_qos_profile_default, false, intra_process);
        bool received = false;
        const auto expected = image();
        auto subscriber = node->create_subscription<sensor_msgs::msg::Image>(
            "~/image", 10, [&](sensor_msgs::msg::Image::ConstSharedPtr message) {
                EXPECT_EQ(*message, expected);
                received = true;
            });
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!received && std::chrono::steady_clock::now() < deadline)
        {
            publisher->publish(cv::Mat(expected.height, expected.width, CV_16UC1,
                const_cast<uint8_t*>(expected.data.data())), expected.encoding, expected.header);
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        EXPECT_TRUE(received);
    }
}

TEST_F(FluxTransport, FluxImageCopiesRowsWithoutSourcePadding)
{
    auto publisher = make_image_publisher(*node, "~/image", rmw_qos_profile_default, true, true);
    flux::Subscription subscriber(*node, "~/image", sensor_msgs::flux_msg::Image::kFingerprint);
    std::array<uint8_t, 16> pixels = {1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 99, 99};
    cv::Mat source(2, 2, CV_8UC3, pixels.data(), 8);
    const auto header = image().header;
    publisher->publish(source, "rgb8", header);
    auto frame = subscriber.take_blocking(1000000000);
    ASSERT_TRUE(frame);
    const auto received = sensor_msgs::flux_msg::frame_to_msg(sensor_msgs::flux_msg::Image::View(frame));
    EXPECT_EQ(received.step, 6U);
    EXPECT_EQ(received.data, (std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12}));
}

TEST_F(FluxTransport, FluxImageGrowsBeyondDefaultSlot)
{
    auto publisher = make_image_publisher(*node, "~/image", rmw_qos_profile_default, true, true);
    cv::Mat source(1200, 2000, CV_8UC4, cv::Scalar(1, 2, 3, 4));
    const auto header = image().header;
    publisher->publish(source, "rgba8", header);
    flux::Subscription subscriber(*node, "~/image", sensor_msgs::flux_msg::Image::kFingerprint);
    publisher->publish(source, "rgba8", header);
    auto frame = subscriber.take_blocking(1000000000);
    ASSERT_TRUE(frame);
    const sensor_msgs::flux_msg::Image::View view(frame);
    ASSERT_EQ(view.data().size(), source.total() * source.elemSize());
    EXPECT_EQ(std::memcmp(view.data().data(), source.data, view.data().size()), 0);
}

TEST_F(FluxTransport, FluxRgbdPreservesImagesAndBothCalibrations)
{
    rgbd_flux_publisher publisher(*node, "~/rgbd");
    flux::Subscription subscriber(*node, "~/rgbd", realsense2_camera_msgs::flux_msg::RGBD::kFingerprint);
    cv::Mat rgb(2, 3, CV_8UC3, cv::Scalar(11, 22, 33));
    cv::Mat depth(2, 3, CV_16UC1, cv::Scalar(1000));
    sensor_msgs::msg::CameraInfo info;
    info.header = image().header;
    info.width = 3;
    info.height = 2;
    info.distortion_model = "plumb_bob";
    info.d = {1, 2, 3, 4, 5};
    info.k = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    info.r = {9, 8, 7, 6, 5, 4, 3, 2, 1};
    info.p = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    info.binning_x = 2;
    info.binning_y = 3;
    info.roi.x_offset = 1;
    info.roi.y_offset = 2;
    info.roi.width = 3;
    info.roi.height = 4;
    info.roi.do_rectify = true;
    auto depth_info = info;
    depth_info.header.frame_id = "aligned_depth_optical_frame";
    depth_info.d = {6, 7, 8};
    depth_info.k[0] = 42;
    publisher.publish(rgb, depth, "rgb8", "16UC1", info, depth_info, info.header);
    auto frame = subscriber.take_blocking(1000000000);
    ASSERT_TRUE(frame);
    const auto received = realsense2_camera_msgs::flux_msg::frame_to_msg(
        realsense2_camera_msgs::flux_msg::RGBD::View(frame));
    EXPECT_EQ(received.header, info.header);
    EXPECT_EQ(received.rgb_camera_info, info);
    EXPECT_EQ(received.depth_camera_info, depth_info);
    EXPECT_EQ(received.rgb, *cv_bridge::CvImage(info.header, "rgb8", rgb).toImageMsg());
    EXPECT_EQ(received.depth, *cv_bridge::CvImage(depth_info.header, "16UC1", depth).toImageMsg());
    EXPECT_EQ(node->count_publishers(subscriber.topic_name()), 0U);
}

TEST_F(FluxTransport, SdkPointcloudMatchesRosForOrderedCompactRgbAndIntensity)
{
    for (bool ordered : {false, true})
    for (rs2_stream texture_stream : {RS2_STREAM_ANY, RS2_STREAM_COLOR, RS2_STREAM_INFRARED})
    {
        const auto frames = synthetic_frames(texture_stream == RS2_STREAM_INFRARED ? RS2_FORMAT_Y8 : RS2_FORMAT_RGB8);
        auto sdk_filter = std::make_shared<rs2::pointcloud>();
        sdk_filter->set_option(RS2_OPTION_STREAM_FILTER, texture_stream);
        if (texture_stream != RS2_STREAM_ANY)
            sdk_filter->map_to(texture_stream == RS2_STREAM_COLOR ? frames.get_color_frame() : frames.get_infrared_frame());
        const auto points = sdk_filter->calculate(frames.get_depth_frame());
        const auto options = rclcpp::NodeOptions().use_intra_process_comms(true).parameter_overrides({
            rclcpp::Parameter("pointcloud.ordered_pc", ordered)});
        auto ros_node = std::make_shared<rclcpp::Node>("ros_camera", options);
        auto flux_node = std::make_shared<rclcpp::Node>("flux_camera", options);
        auto ros_params = std::make_shared<Parameters>(*ros_node);
        auto flux_params = std::make_shared<Parameters>(*flux_node);
        PointcloudFilter ros_filter(sdk_filter, *ros_node, ros_params, ros_node->get_logger(), false, true);
        PointcloudFilter flux_filter(sdk_filter, *flux_node, flux_params, flux_node->get_logger(), true, true);
        ros_filter.setPublisher();
        flux_filter.setPublisher();
        sensor_msgs::msg::PointCloud2::ConstSharedPtr ros_message;
        auto ros_subscriber = ros_node->create_subscription<sensor_msgs::msg::PointCloud2>(
            "~/depth/color/points", 10, [&](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                ros_message = message;
            });
        flux::Subscription flux_subscriber(*flux_node, "~/depth/color/points",
                                           sensor_msgs::flux_msg::PointCloud2::kFingerprint);
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(ros_node);
        const rclcpp::Time stamp(123, 456);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!ros_message && std::chrono::steady_clock::now() < deadline)
        {
            ros_filter.Publish(points, stamp, frames, "depth_optical_frame");
            executor.spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        ASSERT_TRUE(ros_message);
        flux_filter.Publish(points, stamp, frames, "depth_optical_frame");
        auto frame = flux_subscriber.take_blocking(1000000000);
        ASSERT_TRUE(frame);
        const auto message = sensor_msgs::flux_msg::frame_to_msg(sensor_msgs::flux_msg::PointCloud2::View(frame));
        EXPECT_EQ(message, *ros_message);
        EXPECT_EQ(message.width * message.height, ordered ? 16U : 15U);
        EXPECT_EQ(message.point_step, texture_stream == RS2_STREAM_ANY ? 16U : 20U);
        EXPECT_EQ(message.fields.size(), texture_stream == RS2_STREAM_ANY ? 3U : 4U);
        EXPECT_EQ(message.is_dense, !ordered);
        float first_x;
        std::memcpy(&first_x, message.data.data(), sizeof(first_x));
        EXPECT_FLOAT_EQ(first_x, points.get_vertices()[0].x);
        if (texture_stream == RS2_STREAM_COLOR)
        {
            EXPECT_EQ((std::vector<uint8_t>(message.data.begin() + 16, message.data.begin() + 19)),
                      (std::vector<uint8_t>{3, 2, 1}));
        }
    }
}
