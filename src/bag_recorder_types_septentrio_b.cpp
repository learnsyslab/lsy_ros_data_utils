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

#ifdef HAVE_SEPTENTRIO_GNSS_DRIVER
#include <septentrio_gnss_driver/msg/meas_epoch.hpp>
#include <septentrio_gnss_driver/msg/meas_extra.hpp>
#include <septentrio_gnss_driver/msg/ntrip_client_status.hpp>
#include <septentrio_gnss_driver/msg/pvt_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pvt_geodetic.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_cartesian.hpp>
#include <septentrio_gnss_driver/msg/pos_cov_geodetic.hpp>
#include <septentrio_gnss_driver/msg/quality_ind.hpp>
#include <septentrio_gnss_driver/msg/receiver_status.hpp>
#include <septentrio_gnss_driver/msg/receiver_time.hpp>
#include <septentrio_gnss_driver/msg/sat_visibility.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_cartesian.hpp>
#include <septentrio_gnss_driver/msg/vel_cov_geodetic.hpp>
#endif

namespace lsy_ros_data_utils::rosbag {
  void BagRecorderComponent::register_septentrio_types_b() {
#ifdef HAVE_SEPTENTRIO_GNSS_DRIVER
    register_type<septentrio_gnss_driver::msg::MeasEpoch>("septentrio_gnss_driver/msg/MeasEpoch");
    register_type<septentrio_gnss_driver::msg::MeasExtra>("septentrio_gnss_driver/msg/MeasExtra");
    register_type<septentrio_gnss_driver::msg::NTRIPClientStatus>("septentrio_gnss_driver/msg/NTRIPClientStatus");
    register_type<septentrio_gnss_driver::msg::PVTCartesian>("septentrio_gnss_driver/msg/PVTCartesian");
    register_type<septentrio_gnss_driver::msg::PVTGeodetic>("septentrio_gnss_driver/msg/PVTGeodetic");
    register_type<septentrio_gnss_driver::msg::PosCovCartesian>("septentrio_gnss_driver/msg/PosCovCartesian");
    register_type<septentrio_gnss_driver::msg::PosCovGeodetic>("septentrio_gnss_driver/msg/PosCovGeodetic");
    register_type<septentrio_gnss_driver::msg::QualityInd>("septentrio_gnss_driver/msg/QualityInd");
    register_type<septentrio_gnss_driver::msg::ReceiverStatus>("septentrio_gnss_driver/msg/ReceiverStatus");
    register_type<septentrio_gnss_driver::msg::ReceiverTime>("septentrio_gnss_driver/msg/ReceiverTime");
    register_type<septentrio_gnss_driver::msg::SatVisibility>("septentrio_gnss_driver/msg/SatVisibility");
    register_type<septentrio_gnss_driver::msg::VelCovCartesian>("septentrio_gnss_driver/msg/VelCovCartesian");
    register_type<septentrio_gnss_driver::msg::VelCovGeodetic>("septentrio_gnss_driver/msg/VelCovGeodetic");
#endif
  }
}
