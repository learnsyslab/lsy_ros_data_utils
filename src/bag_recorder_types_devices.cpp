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

#ifdef HAVE_LIVOX_ROS_DRIVER2
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/time_sync_status.hpp>
#endif

#ifdef HAVE_ZED_MSGS
#include <zed_msgs/msg/health_status_stamped.hpp>
#include <zed_msgs/msg/heartbeat.hpp>
#include <zed_msgs/msg/pos_track_status.hpp>
#endif

#ifdef HAVE_TI_MMWAVE_ROS2_INTERFACES
#include <ti_mmwave_ros2_interfaces/msg/radar_scan.hpp>
#endif

namespace lsy_ros_data_utils::rosbag {
  void BagRecorderComponent::register_device_types() {
#ifdef HAVE_LIVOX_ROS_DRIVER2
    register_type<livox_ros_driver2::msg::CustomMsg>("livox_ros_driver2/msg/CustomMsg");
    register_type<livox_ros_driver2::msg::TimeSyncStatus>("livox_ros_driver2/msg/TimeSyncStatus");
#endif
#ifdef HAVE_ZED_MSGS
    register_type<zed_msgs::msg::HealthStatusStamped>("zed_msgs/msg/HealthStatusStamped");
    register_type<zed_msgs::msg::Heartbeat>("zed_msgs/msg/Heartbeat");
    register_type<zed_msgs::msg::PosTrackStatus>("zed_msgs/msg/PosTrackStatus");
#endif
#ifdef HAVE_TI_MMWAVE_ROS2_INTERFACES
    register_type<ti_mmwave_ros2_interfaces::msg::RadarScan>("ti_mmwave_ros2_interfaces/msg/RadarScan");
#endif
  }
}
