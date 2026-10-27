#pragma once

#include "ros_node_base.h"
#include <flux/publisher.hpp>
#include <sensor_msgs/flux/image.hpp>
#include <sensor_msgs/flux/point_cloud2.hpp>
#include <memory>
#include <string>

namespace realsense2_camera
{
template<class Adapter>
class flux_publisher
{
public:
    flux_publisher(RosNodeBase& node, const std::string& topic) :
        _node(node), _topic(topic),
        _publisher(std::make_unique<flux::Publisher>(node, topic, Adapter::kFingerprint))
    {}

    typename Adapter::Builder build(size_t required)
    {
        uint32_t capacity = _publisher->slot_size();
        if (required > capacity)
        {
            while (capacity < required)
                capacity *= 2;
            _publisher.reset();
            _publisher = std::make_unique<flux::Publisher>(_node, _topic, Adapter::kFingerprint, capacity);
        }
        return Adapter::build__(*_publisher);
    }

    void publish(typename Adapter::Builder& builder)
    {
        const auto result = builder.commit__();
        if (flux::faulted(result))
            RCLCPP_ERROR_THROTTLE(_node.get_logger(), *_node.get_clock(), 5000,
                                 "Flux publish failed on %s: %s", _topic.c_str(), flux::to_string(result));
    }

private:
    RosNodeBase& _node;
    std::string _topic;
    std::unique_ptr<flux::Publisher> _publisher;
};
}
