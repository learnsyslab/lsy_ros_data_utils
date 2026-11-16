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
#include <septentrio_gnss_driver/msg/att_cov_euler.hpp>
#include <septentrio_gnss_driver/msg/att_euler.hpp>
#include <septentrio_gnss_driver/msg/bds_ion.hpp>
#include <septentrio_gnss_driver/msg/bds_nav.hpp>
#include <septentrio_gnss_driver/msg/bds_utc.hpp>
#include <septentrio_gnss_driver/msg/diff_corr_in.hpp>
#include <septentrio_gnss_driver/msg/gal_ion.hpp>
#include <septentrio_gnss_driver/msg/gal_nav.hpp>
#include <septentrio_gnss_driver/msg/gal_utc.hpp>
#include <septentrio_gnss_driver/msg/glo_nav.hpp>
#include <septentrio_gnss_driver/msg/glo_time.hpp>
#include <septentrio_gnss_driver/msg/gps_ion.hpp>
#include <septentrio_gnss_driver/msg/gps_nav.hpp>
#include <septentrio_gnss_driver/msg/gps_utc.hpp>
#endif

namespace lsy_ros_data_utils::rosbag {
  void BagRecorderComponent::register_septentrio_types_a() {
#ifdef HAVE_SEPTENTRIO_GNSS_DRIVER
    register_type<septentrio_gnss_driver::msg::AttCovEuler>("septentrio_gnss_driver/msg/AttCovEuler");
    register_type<septentrio_gnss_driver::msg::AttEuler>("septentrio_gnss_driver/msg/AttEuler");
    register_type<septentrio_gnss_driver::msg::BDSIon>("septentrio_gnss_driver/msg/BDSIon");
    register_type<septentrio_gnss_driver::msg::BDSNav>("septentrio_gnss_driver/msg/BDSNav");
    register_type<septentrio_gnss_driver::msg::BDSUtc>("septentrio_gnss_driver/msg/BDSUtc");
    register_type<septentrio_gnss_driver::msg::DiffCorrIn>("septentrio_gnss_driver/msg/DiffCorrIn");
    register_type<septentrio_gnss_driver::msg::GALIon>("septentrio_gnss_driver/msg/GALIon");
    register_type<septentrio_gnss_driver::msg::GALNav>("septentrio_gnss_driver/msg/GALNav");
    register_type<septentrio_gnss_driver::msg::GALUtc>("septentrio_gnss_driver/msg/GALUtc");
    register_type<septentrio_gnss_driver::msg::GLONav>("septentrio_gnss_driver/msg/GLONav");
    register_type<septentrio_gnss_driver::msg::GLOTime>("septentrio_gnss_driver/msg/GLOTime");
    register_type<septentrio_gnss_driver::msg::GPSIon>("septentrio_gnss_driver/msg/GPSIon");
    register_type<septentrio_gnss_driver::msg::GPSNav>("septentrio_gnss_driver/msg/GPSNav");
    register_type<septentrio_gnss_driver::msg::GPSUtc>("septentrio_gnss_driver/msg/GPSUtc");
#endif
  }
}
