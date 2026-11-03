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

//
// Created by haoming on 1/17/26.
//

#ifndef LSY_ROS_DATA_UTILS_UTILS_HPP
#define LSY_ROS_DATA_UTILS_UTILS_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <sstream>
#include <ctime>

namespace lsy_ros_data_utils {
  // -------------- Output utils --------------

  inline std::string bar(const double value, double ref, const int width) {
    if (ref <= 0.0) ref = 1.0;
    const double ratio = std::clamp(value / ref, 0.0, 1.0);
    const int filled = static_cast<int>(ratio * width + 0.5); // round nicely

    return std::string(static_cast<size_t>(filled), '#') +
           std::string(static_cast<size_t>(width - filled), '-');
  }

  inline std::string maybe_red(const std::string &s, const bool color = true) {
    return color ? std::string("\033[31m") + s + "\033[0m" : s;
  }

  inline std::string maybe_yellow(const std::string &s, const bool color = true) {
    return color ? std::string("\033[33m") + s + "\033[0m" : s;
  }

  // -------------- Date & Time utils --------------
  inline std::string now_string_local() {
    using clock = std::chrono::system_clock;
    const auto tp = clock::now();

    const auto t = clock::to_time_t(tp);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

    // milliseconds
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()) % 1000;
    oss << "." << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
  }

  using SteadyClock = std::chrono::steady_clock;

  inline int64_t now_ns_steady() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
          SteadyClock::now().time_since_epoch())
        .count();
  }

  // Lock-free rate limiter: returns true at most once per period across all threads.
  inline bool should_warn(std::atomic<int64_t> &last_ns, const int64_t now_ns, const double period_sec) {
    const auto period_ns = static_cast<int64_t>(period_sec * 1e9);
    int64_t prev = last_ns.load(std::memory_order_relaxed);
    if (prev != 0 && now_ns - prev < period_ns) return false;
    return last_ns.compare_exchange_strong(prev, now_ns, std::memory_order_relaxed);
  }

  // ------------------  ROS utils  ---------------------

  inline rclcpp::NodeOptions ensureIntraProcesComm(const rclcpp::NodeOptions &options) {
    auto new_options = options;
    new_options.use_intra_process_comms(true);
    return new_options;
  }

  // Accepted names: "sensor_data"/"sensor" (best effort) and "default"/"reliable" (reliable).
  // Returns the normalized name ("sensor_data" or "default"); throws on unknown names.
  inline std::string normalize_qos_name(const std::string &qos_name) {
    if (qos_name == "sensor_data" || qos_name == "sensor") return "sensor_data";
    if (qos_name == "default" || qos_name == "reliable") return "default";
    throw std::invalid_argument("Unknown qos '" + qos_name +
                                "' (expected 'sensor_data', 'sensor', 'default' or 'reliable')");
  }

  inline rclcpp::QoS qos_from_string(const std::string &qos_name, const size_t depth = 0) {
    const auto name = normalize_qos_name(qos_name);
    if (name == "sensor_data") {
      auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
      if (depth > 0) qos.keep_last(depth);
      return qos;
    }
    return rclcpp::QoS(rclcpp::KeepLast(depth > 0 ? depth : 10));
  }

  template<typename T, typename = void>
  struct has_header : std::false_type {
  };

  template<typename T>
  struct has_header<T, std::void_t<decltype(std::declval<T>().header.stamp)> > : std::true_type {
  };

  template<typename MsgT>
  int64_t get_message_timestamp_ns(const MsgT &msg, const int64_t recv_ns) {
    if constexpr (has_header<MsgT>::value) {
      return static_cast<int64_t>(msg.header.stamp.sec) * 1000000000LL +
             static_cast<int64_t>(msg.header.stamp.nanosec);
    } else {
      // fallback: receipt time
      return recv_ns;
    }
  }

  inline std::string require_string(const YAML::Node &n, const char *key) {
    if (!n[key] || !n[key].IsScalar()) {
      throw std::runtime_error(std::string("Missing/invalid key: ") + key);
    }
    return n[key].as<std::string>();
  }

  template<typename T>
  T get_or(const YAML::Node &n, const char *key, const T &fallback) {
    return n[key] ? n[key].as<T>() : fallback;
  }
}

#endif //LSY_ROS_DATA_UTILS_UTILS_HPP
