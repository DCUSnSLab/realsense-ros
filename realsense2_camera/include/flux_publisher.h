#pragma once

#include "ros_node_base.h"
#include <flux/publisher.hpp>
#include <sensor_msgs/flux/image.hpp>
#include <sensor_msgs/flux/point_cloud2.hpp>
#include <cstdint>
#include <string>

namespace realsense2_camera
{
struct flux_channel_config
{
    uint32_t slot_size;
    uint32_t slot_count;
};

struct flux_config
{
    bool use_flux;
    flux_channel_config image;
    flux_channel_config rgbd;
    flux_channel_config pointcloud;
};

flux_config load_flux_config(const std::string& path);

template<class Adapter>
class flux_publisher
{
public:
    flux_publisher(RosNodeBase& node, const std::string& topic, const flux_channel_config& config) :
        _node(node),
        _publisher(node, topic, Adapter::kFingerprint, config.slot_size, config.slot_count)
    {}

    typename Adapter::Builder build()
    {
        return Adapter::build__(_publisher);
    }

    void publish(typename Adapter::Builder& builder)
    {
        const auto result = builder.commit__();
        if (flux::faulted(result))
            RCLCPP_ERROR_THROTTLE(_node.get_logger(), *_node.get_clock(), 5000,
                                 "Flux publish failed on %s: %s (slot_size %u bytes)",
                                 _publisher.topic_name().c_str(), flux::to_string(result),
                                 _publisher.slot_size());
    }

private:
    RosNodeBase& _node;
    flux::Publisher _publisher;
};
}
