// Copyright (c) 2026  Learning Systems and Robotics Lab,
// Technical University of Munich (TUM)
//
// Authors:  Haoming Zhang <haoming.zhang@tum>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//


// Typed subscription support for the bag recorder, part of a set of files:
// see the note at BagRecorderComponent::init_type_registry() in the header.
// Only types published by components that share the recorder's container are
// worth registering (that is where a typed subscription saves a copy); any
// other type is recorded through a generic subscription without registration.
// Config strings must match exactly.

#include "bag_recorder_component.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#ifdef HAVE_GPS_MSGS
#include <gps_msgs/msg/gps_fix.hpp>
#endif

namespace lsy_ros_data_utils::rosbag {
  void BagRecorderComponent::register_common_types() {
    register_type<sensor_msgs::msg::Image>("sensor_msgs/msg/Image");
    register_type<sensor_msgs::msg::CameraInfo>("sensor_msgs/msg/CameraInfo");
    register_type<sensor_msgs::msg::Imu>("sensor_msgs/msg/Imu");
    register_type<sensor_msgs::msg::PointCloud2>("sensor_msgs/msg/PointCloud2");
    register_type<sensor_msgs::msg::Temperature>("sensor_msgs/msg/Temperature");
    register_type<sensor_msgs::msg::NavSatFix>("sensor_msgs/msg/NavSatFix");
    register_type<sensor_msgs::msg::TimeReference>("sensor_msgs/msg/TimeReference");
    register_type<geometry_msgs::msg::PoseStamped>("geometry_msgs/msg/PoseStamped");
    register_type<geometry_msgs::msg::TransformStamped>("geometry_msgs/msg/TransformStamped");
    register_type<nav_msgs::msg::Odometry>("nav_msgs/msg/Odometry");
#ifdef HAVE_GPS_MSGS
    register_type<gps_msgs::msg::GPSFix>("gps_msgs/msg/GPSFix");
#endif
  }
}
