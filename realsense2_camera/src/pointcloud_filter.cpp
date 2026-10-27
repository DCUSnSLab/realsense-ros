// Copyright 2025 Intel Corporation. All Rights Reserved.
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

#include <pointcloud_filter.h>
#include <fstream>
#include <array>
#include <cstring>
#include <sensor_msgs/point_cloud2_iterator.hpp>


using namespace realsense2_camera;


PointcloudFilter::PointcloudFilter(std::shared_ptr<rs2::filter> filter, RosNodeBase& node, std::shared_ptr<Parameters> parameters, rclcpp::Logger logger, bool use_flux, bool is_enabled):
    NamedFilter(filter, parameters, logger, is_enabled, false),
    _node(node),
    _allow_no_texture_points(ALLOW_NO_TEXTURE_POINTS),
    _ordered_pc(ORDERED_PC),
    _use_flux(use_flux)
    {
        setParameters();
    }

void PointcloudFilter::setParameters()
{
    std::string module_name = create_graph_resource_name(rs2_to_ros(_filter->get_info(RS2_CAMERA_INFO_NAME)));
    std::string param_name(module_name + "." + "allow_no_texture_points");
    _params.getParameters()->setParamT(param_name, _allow_no_texture_points);
    _parameters_names.push_back(param_name);

    param_name = module_name + "." + std::string("ordered_pc");
    _params.getParameters()->setParamT(param_name, _ordered_pc);
    _parameters_names.push_back(param_name);

    param_name = module_name + "." + std::string("pointcloud_qos");
    rcl_interfaces::msg::ParameterDescriptor crnt_descriptor;
    crnt_descriptor.description = "Available options are:\n" + list_available_qos_strings();
    _pointcloud_qos = _params.getParameters()->setParam<std::string>(param_name, DEFAULT_QOS, [this](const rclcpp::Parameter& parameter)
            {
                try
                {
                    qos_string_to_qos(parameter.get_value<std::string>());
                    _pointcloud_qos = parameter.get_value<std::string>();
                    ROS_WARN_STREAM("re-enable the stream for the change to take effect.");
                }
                catch(const std::exception& e)
                {
                    ROS_ERROR_STREAM("Given value, " << parameter.get_value<std::string>() << " is unknown. Set ROS param back to: " << _pointcloud_qos);
                    _params.getParameters()->queueSetRosValue(parameter.get_name(), _pointcloud_qos);
                }
            }, crnt_descriptor);
    _parameters_names.push_back(param_name);
    NamedFilter::setParameters([this](const rclcpp::Parameter& )
        {
            setPublisher();
        });
}

void PointcloudFilter::setPublisher()
{
    std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
    if (_use_flux)
    {
        if (_is_enabled && !_flux_pointcloud_publisher)
            _flux_pointcloud_publisher = std::make_unique<flux_publisher<sensor_msgs::flux_msg::PointCloud2>>(
                _node, "~/depth/color/points");
        else if (!_is_enabled)
            _flux_pointcloud_publisher.reset();
        return;
    }
    if ((_is_enabled) && (!_pointcloud_publisher))
    {
        _pointcloud_publisher = _node.create_publisher<sensor_msgs::msg::PointCloud2>("~/depth/color/points",
                                rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_string_to_qos(_pointcloud_qos)),
                                            qos_string_to_qos(_pointcloud_qos)));
    }
    else if ((!_is_enabled) && (_pointcloud_publisher))
    {
        _pointcloud_publisher.reset();
    }
}

void reverse_memcpy(unsigned char* dst, const unsigned char* src, size_t n)
{
    size_t i;

    for (i=0; i < n; ++i)
        dst[n-1-i] = src[i];

}

void PointcloudFilter::Publish(rs2::points pc, const rclcpp::Time& t, const rs2::frameset& frameset, const std::string& frame_id)
{
    {
        std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
        if (_use_flux ? !_flux_pointcloud_publisher :
            ((!_pointcloud_publisher) || (!(_pointcloud_publisher->get_subscription_count()))))
            return;
    }
    rs2_stream texture_source_id = static_cast<rs2_stream>(_filter->get_option(rs2_option::RS2_OPTION_STREAM_FILTER));
    bool use_texture = texture_source_id != RS2_STREAM_ANY;
    static int warn_count(0);
    static const int DISPLAY_WARN_NUMBER(5);
    rs2::frameset::iterator texture_frame_itr = frameset.end();
    if (use_texture)
    {
        std::set<rs2_format> available_formats{ rs2_format::RS2_FORMAT_RGB8, rs2_format::RS2_FORMAT_Y8 };

        texture_frame_itr = std::find_if(frameset.begin(), frameset.end(), [&texture_source_id, &available_formats] (rs2::frame f)
                                {return (rs2_stream(f.get_profile().stream_type()) == texture_source_id) &&
                                            (available_formats.find(f.get_profile().format()) != available_formats.end()); });
        if (texture_frame_itr == frameset.end())
        {
            warn_count++;
            std::string texture_source_name = _filter->get_option_value_description(rs2_option::RS2_OPTION_STREAM_FILTER, static_cast<float>(texture_source_id));
            ROS_WARN_STREAM_COND(warn_count == DISPLAY_WARN_NUMBER, "No stream match for pointcloud chosen texture " << texture_source_name);
            return;
        }
        warn_count = 0;
    }

    rs2::video_frame texture(nullptr);
    if (use_texture)
        texture = (*texture_frame_itr).as<rs2::video_frame>();
    const bool rgb = texture && texture.get_profile().format() == RS2_FORMAT_RGB8;
    const std::string texture_name = rgb ? "rgb" : "intensity";
    const uint32_t point_step = use_texture ? 20 : 16;
    const size_t field_count = use_texture ? 4 : 3;
    const rs2::vertex* vertices = pc.get_vertices();
    const rs2::texture_coordinate* coordinates = pc.get_texture_coordinates();
    const auto valid_texture = [&](size_t index) {
        const auto& uv = coordinates[index];
        return uv.u >= 0.f && uv.u <= 1.f && uv.v >= 0.f && uv.v <= 1.f;
    };
    const auto keep_point = [&](size_t index) {
        return _ordered_pc || (vertices[index].z > 0 &&
            (!use_texture || valid_texture(index) || _allow_no_texture_points));
    };
    size_t count = 0;
    for (size_t index = 0; index < pc.size(); ++index)
        count += keep_point(index);
    const auto intrinsics = pc.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    const uint32_t width = _ordered_pc ? intrinsics.width : count;
    const uint32_t height = _ordered_pc ? intrinsics.height : 1;
    const size_t nbytes = count * point_step;
    const auto fill_data = [&](uint8_t* output) {
        if (nbytes == 0)
            return;
        std::fill_n(output, nbytes, 0);
        for (size_t index = 0; index < pc.size(); ++index)
        {
            if (!keep_point(index))
                continue;
            std::memcpy(output, &vertices[index], 3 * sizeof(float));
            if (use_texture && valid_texture(index))
            {
                const int x = std::min(static_cast<int>(coordinates[index].u * texture.get_width()), texture.get_width() - 1);
                const int y = std::min(static_cast<int>(coordinates[index].v * texture.get_height()), texture.get_height() - 1);
                const auto* pixel = static_cast<const uint8_t*>(texture.get_data()) +
                    y * texture.get_stride_in_bytes() + x * texture.get_bytes_per_pixel();
                reverse_memcpy(output + 16, pixel, texture.get_bytes_per_pixel());
            }
            output += point_step;
        }
    };
    const std::array<std::string, 4> names = {"x", "y", "z", texture_name};
    std::lock_guard<std::mutex> lock_guard(_mutex_publisher);
    if (_use_flux)
    {
        if (!_flux_pointcloud_publisher)
            return;
        size_t required = sensor_msgs::flux_msg::PointCloud2::kScalarBytes + frame_id.size() +
            3 + field_count * sensor_msgs::flux_msg::PointCloud2::FieldsElem::kStride + nbytes;
        for (size_t index = 0; index < field_count; ++index)
            required += names[index].size();
        auto builder = _flux_pointcloud_publisher->build(required);
        if (!builder)
            return;
        const builtin_interfaces::msg::Time stamp = t;
        builder.set__header__stamp(stamp.sec, stamp.nanosec);
        builder.set__header__frame_id(frame_id);
        builder.set__height(height);
        builder.set__width(width);
        auto fields = builder.alloc__fields(field_count);
        for (size_t index = 0; index < field_count; ++index)
        {
            auto field = fields[index];
            field.set__name(names[index]);
            field.set__offset(index == 3 ? 16 : index * 4);
            field.set__datatype(sensor_msgs::msg::PointField::FLOAT32);
            field.set__count(1);
        }
        builder.set__is_bigendian(false);
        builder.set__point_step(point_step);
        builder.set__row_step(width * point_step);
        builder.set__is_dense(!_ordered_pc);
        auto data = builder.alloc__data(nbytes);
        if (data.size() == nbytes)
            fill_data(data.data());
        _flux_pointcloud_publisher->publish(builder);
    }
    else if (_pointcloud_publisher)
    {
        auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
        sensor_msgs::PointCloud2Modifier modifier(*message);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        if (use_texture)
            message->point_step = addPointField(*message, texture_name.c_str(), 1,
                sensor_msgs::msg::PointField::FLOAT32, message->point_step);
        message->header.stamp = t;
        message->header.frame_id = frame_id;
        message->width = width;
        message->height = height;
        message->row_step = width * point_step;
        message->is_dense = !_ordered_pc;
        message->data.resize(nbytes);
        fill_data(message->data.data());
        _pointcloud_publisher->publish(std::move(message));
    }
}
