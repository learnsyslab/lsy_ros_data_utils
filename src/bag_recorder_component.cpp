// Copyright (c) 2026  Learning Systems and Robotics Lab,
// Technical University of Munich (TUM)
//
// Authors:  Haoming Zhang <haoming.zhang@tum.de>
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


#include "bag_recorder_component.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <set>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/storage_options.hpp>

namespace {
  constexpr size_t kGiB = 1ull << 30;

  // Prefer new API (send_timestamp/recv_timestamp) if it exists
  template<typename M>
  auto set_ts_impl(M &m, int64_t send_ns, int64_t recv_ns, int)
    -> decltype((m.send_timestamp = send_ns, m.recv_timestamp = recv_ns), void()) {
    m.send_timestamp = send_ns;
    m.recv_timestamp = recv_ns;
  }

  // Fallback to old API (time_stamp) if it exists
  template<typename M>
  auto set_ts_impl(M &m, int64_t /*send_ns*/, int64_t recv_ns, long)
    -> decltype((m.time_stamp = recv_ns), void()) {
    m.time_stamp = recv_ns;
  }

  size_t gib_to_bytes(const double gib) {
    if (gib < 0.0) throw std::runtime_error("size in GiB must be >= 0");
    return static_cast<size_t>(gib * static_cast<double>(kGiB));
  }

  const char *policy_name(const lsy_ros_data_utils::rosbag::BagSpec::OverflowPolicy p) {
    return p == lsy_ros_data_utils::rosbag::BagSpec::OverflowPolicy::BLOCK ? "BLOCK" : "DROP_OLDEST";
  }

  bool is_image_type(const std::string &type) { return type == "sensor_msgs/msg/Image"; }
}

namespace lsy_ros_data_utils::rosbag {
  // ---------------- helpers ----------------

  void BagRecorderComponent::set_bag_message_timestamps(
    rosbag2_storage::SerializedBagMessage &m, const int64_t send_ns, const int64_t recv_ns) {
    // int overload is preferred; if not viable, falls back to long overload.
    set_ts_impl(m, send_ns, recv_ns, 0);
  }

  BagRecorderComponent::PayloadPtr BagRecorderComponent::to_payload(rclcpp::SerializedMessage &&serialized) {
    // Zero-copy: keep the SerializedMessage alive and alias its rcl buffer.
    auto owner = std::make_shared<rclcpp::SerializedMessage>(std::move(serialized));
    return {owner, &owner->get_rcl_serialized_message()};
  }

  // ---------------- main ----------------

  BagRecorderComponent::BagRecorderComponent(const rclcpp::NodeOptions &node_options)
    : rclcpp::Node("bag_recorder_component", ensureIntraProcesComm(node_options)) {
    this->declare_parameter<std::string>("config_file", "");
    cfg_file_ = this->get_parameter("config_file").as_string();
    if (cfg_file_.empty()) {
      throw std::runtime_error("Parameter 'config_file' is required.");
    }

    this->load_config(cfg_file_);
    this->init_type_registry();
    this->build_routes();

    for (auto &br: bags_) {
      this->open_bag_and_create_topics(*br);
    }

    // one subscription per unique topic
    this->create_subscriptions();

    // writer thread per bag
    for (auto &br: bags_) {
      BagRuntime *raw = br.get();
      raw->thread = std::thread([this, raw]() { this->writer_loop(*raw); });
    }

    // On Ctrl-C (rclcpp::shutdown) start draining and closing the bags immediately,
    // instead of waiting for the container to destroy the component.
    context_ = this->get_node_base_interface()->get_context();
    shutdown_cb_handle_ = context_->add_on_shutdown_callback([this]() { this->request_stop(); });

    if (monitor_cfg_.enabled && monitor_cfg_.print_hz > 0.0) {
      const auto period = std::chrono::duration<double>(1.0 / monitor_cfg_.print_hz);
      auto life = life_;
      monitor_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this, life]() {
          const CallbackScope scope(*life);
          if (scope.ok()) this->monitor_timer_cb();
        });
    }

    RCLCPP_INFO(get_logger(), "BagRecorderComponent started. bags=%zu unique_topics=%zu subscriptions=%zu",
                bags_.size(), routes_.size(), subs_.size());
  }

  BagRecorderComponent::~BagRecorderComponent() {
    if (context_ && shutdown_cb_handle_) {
      context_->remove_on_shutdown_callback(*shutdown_cb_handle_);
    }

    // 1. refuse new callbacks, wake up blocked producers, tell writers to drain
    request_stop();
    if (monitor_timer_) monitor_timer_->cancel();

    // 2. wait for callbacks that are still executing on other executor threads
    const auto t0 = std::chrono::steady_clock::now();
    auto next_log = t0 + std::chrono::seconds(2);
    while (life_->active_callbacks.load() > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (std::chrono::steady_clock::now() > next_log) {
        RCLCPP_WARN(get_logger(), "Waiting for %d in-flight callback(s) to finish...",
                    life_->active_callbacks.load());
        next_log += std::chrono::seconds(2);
      }
    }
    subs_.clear();
    monitor_timer_.reset();

    // 3. writers drain their queues and close the bags
    for (const auto &br: bags_) {
      if (br->thread.joinable()) br->thread.join();
    }
    RCLCPP_INFO(get_logger(), "BagRecorderComponent destroyed, all bags closed.");
  }

  void BagRecorderComponent::request_stop() {
    life_->shutting_down.store(true);
    for (const auto &br: bags_) {
      {
        // set under the lock, otherwise a waiter can miss the wake-up
        std::lock_guard<std::mutex> lk(br->mtx);
        br->stop = true;
      }
      br->cv_not_empty.notify_all();
      br->cv_not_full.notify_all();
    }
  }

  // ---------------- config ----------------

  void BagRecorderComponent::load_config(const std::string &yaml_path) {
    const YAML::Node config = YAML::LoadFile(yaml_path);
    if (!config["bags"] || !config["bags"].IsSequence()) {
      throw std::runtime_error("YAML must contain 'bags' as a sequence.");
    }

    // callback groups
    if (!config["num_callback_groups"]) {
      throw std::runtime_error("Parameter 'num_callback_groups' required!");
    }
    const auto num_callback_groups = config["num_callback_groups"].as<size_t>();
    if (num_callback_groups == 0) {
      throw std::runtime_error("'num_callback_groups' must be >= 1");
    }
    cb_groups_.clear();
    for (size_t i = 0; i < num_callback_groups; ++i) {
      cb_groups_.push_back(this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
    }

    // monitor (optional)
    monitor_cfg_ = MonitorCfg{};
    if (const auto m = config["monitor"]) {
      monitor_cfg_.enabled = get_or<bool>(m, "enabled", true);
      monitor_cfg_.print_hz = get_or<double>(m, "print_hz", 1.0);
      monitor_cfg_.ewma_tau_sec = get_or<double>(m, "ewma_tau_sec", 2.0);
      monitor_cfg_.warn_grace_sec = get_or<double>(m, "warn_grace_sec", 2.0);
      monitor_cfg_.use_ansi_color = get_or<bool>(m, "use_ansi_color", true);
      monitor_cfg_.clear_screen = get_or<bool>(m, "clear_screen", true);
    }

    static const auto package_config_path =
        ament_index_cpp::get_package_share_directory("lsy_ros_data_utils") + "/config/";

    bags_.clear();
    for (const auto &bag: config["bags"]) {
      auto br = std::make_unique<BagRuntime>();
      BagSpec &spec = br->spec;
      spec.name = require_string(bag, "name");
      spec.bag_uri = require_string(bag, "bag_uri");
      spec.storage_id = get_or<std::string>(bag, "storage_id", "mcap");
      spec.uri_with_datetime = get_or<bool>(bag, "uri_with_datetime", true);

      spec.max_queue_size = get_or<size_t>(bag, "max_queue_size", 20000);
      spec.max_queue_bytes = get_or<size_t>(bag, "max_queue_bytes", 1ull << 30); // bytes
      if (bag["bag_cache_size"]) spec.bag_cache_size = gib_to_bytes(bag["bag_cache_size"].as<double>()); // GiB
      if (bag["max_bag_size"]) spec.max_bag_size = gib_to_bytes(bag["max_bag_size"].as<double>()); // GiB

      const auto policy = get_or<std::string>(bag, "overflow_policy", "drop_oldest");
      if (policy == "block") {
        spec.overflow_policy = BagSpec::OverflowPolicy::BLOCK;
      } else if (policy == "drop_oldest") {
        spec.overflow_policy = BagSpec::OverflowPolicy::DROP_OLDEST;
      } else {
        throw std::runtime_error("Bag '" + spec.name + "': unknown overflow_policy '" + policy +
                                 "' (expected 'drop_oldest' or 'block')");
      }
      spec.warn_on_overflow = get_or<bool>(bag, "warn_on_overflow", true);
      spec.overflow_warn_period_sec = get_or<double>(bag, "overflow_warn_period_sec", 1.0);

      if (bag["storage_config_file"]) {
        const auto f = bag["storage_config_file"].as<std::string>();
        spec.storage_config_uri = (!f.empty() && f.front() == '/') ? f : package_config_path + f;
      }

      // image compression (per bag)
      spec.compress_images = get_or<bool>(bag, "compress_image", false);
      spec.compression.jpeg_quality = get_or<int>(bag, "jpeg_quality", 95);
      spec.compression.png_level = get_or<int>(bag, "png_level", 3);
      const auto fmt = get_or<std::string>(bag, "compression_type", "png");
      spec.compression.format = normalize_compression_format(fmt);
      if (spec.compress_images && spec.compression.format.empty()) {
        throw std::runtime_error("Bag '" + spec.name + "': unsupported compression_type '" + fmt +
                                 "' (expected 'png' or 'jpeg')");
      }

      if (!bag["topics"] || !bag["topics"].IsSequence()) {
        throw std::runtime_error("Bag '" + spec.name + "' must contain 'topics' as a sequence.");
      }

      std::set<std::string> names_in_bag; // names as written into the bag
      for (const auto &t: bag["topics"]) {
        TopicSpec ts;
        ts.name = require_string(t, "name");
        ts.type = require_string(t, "type");
        try {
          ts.qos = normalize_qos_name(get_or<std::string>(t, "qos", "default"));
        } catch (const std::exception &e) {
          throw std::runtime_error("Topic '" + ts.name + "': " + e.what());
        }
        ts.qos_depth = get_or<size_t>(t, "qos_depth", 0);
        ts.mode = get_or<std::string>(t, "mode", "auto");
        if (ts.mode != "auto" && ts.mode != "typed" && ts.mode != "generic") {
          throw std::runtime_error("Topic '" + ts.name + "': unknown mode '" + ts.mode +
                                   "' (expected 'auto', 'typed' or 'generic')");
        }
        if (const auto mon = t["monitor"]) {
          ts.monitor_enabled = true;
          ts.expected_hz = get_or<double>(mon, "expected_hz", 0.0);
          ts.warn_below_hz = get_or<double>(mon, "warn_below_hz", 0.0);
        }
        if (!t["callback_group_id"] || t["callback_group_id"].as<size_t>() >= cb_groups_.size()) {
          throw std::runtime_error("Topic '" + ts.name + "' must have a callback_group_id in [0, " +
                                   std::to_string(cb_groups_.size() - 1) + "]");
        }
        ts.cb_group = cb_groups_[t["callback_group_id"].as<size_t>()];

        const std::string bag_name =
            (spec.compress_images && is_image_type(ts.type)) ? ts.name + "/compressed" : ts.name;
        if (!names_in_bag.insert(bag_name).second) {
          throw std::runtime_error("Bag '" + spec.name + "': topic '" + bag_name +
                                   "' appears more than once (messages would be duplicated)");
        }
        spec.topics.push_back(std::move(ts));
      }

      bags_.push_back(std::move(br));
    }
  }

  void BagRecorderComponent::build_routes() {
    routes_.clear();
    topic_stats_.clear();

    for (const auto &br_ptr: bags_) {
      BagRuntime &br = *br_ptr;
      for (const auto &ts: br.spec.topics) {
        auto &slot = routes_[ts.name];
        if (!slot) {
          slot = std::make_unique<TopicRoute>();
          slot->topic = ts.name;
          slot->type = ts.type;
          slot->compressed_topic = ts.name + "/compressed";
          slot->qos = ts.qos;
          slot->qos_depth = ts.qos_depth;
          slot->mode = ts.mode;
          slot->cb_group = ts.cb_group;
        } else {
          TopicRoute &r = *slot;
          if (r.type != ts.type) {
            throw std::runtime_error("Topic '" + ts.name + "' has conflicting types: '" + r.type + "' vs '" +
                                     ts.type + "'");
          }
          // if ANY bag wants sensor_data => best effort (matches both reliable and best-effort publishers)
          if (ts.qos == "sensor_data") r.qos = "sensor_data";
          r.qos_depth = std::max(r.qos_depth, ts.qos_depth);
          // explicit beats auto; typed vs generic is a conflict
          if (r.mode == "auto") {
            r.mode = ts.mode;
          } else if (ts.mode != "auto" && ts.mode != r.mode) {
            throw std::runtime_error("Topic '" + ts.name + "' has conflicting modes across bags: '" + r.mode +
                                     "' vs '" + ts.mode + "'");
          }
          if (r.cb_group != ts.cb_group) {
            RCLCPP_WARN(get_logger(), "Topic '%s' is assigned to different callback groups across bags; "
                        "using the first one.", ts.name.c_str());
          }
        }

        TopicRoute &r = *slot;
        if (br.spec.compress_images && is_image_type(ts.type)) {
          if (!r.compressed_bags.empty() && !(r.compression == br.spec.compression)) {
            throw std::runtime_error("Topic '" + ts.name +
                                     "' is compressed by several bags with different compression settings");
          }
          r.compression = br.spec.compression;
          r.compressed_bags.push_back(&br);
        } else {
          r.raw_bags.push_back(&br);
        }

        if (ts.monitor_enabled) {
          auto [it, inserted] = topic_stats_.try_emplace(ts.name);
          TopicStats &st = it->second;
          if (inserted) {
            st.expected_hz = ts.expected_hz;
            st.warn_below_hz = ts.warn_below_hz;
          } else if (st.expected_hz != ts.expected_hz || st.warn_below_hz != ts.warn_below_hz) {
            throw std::runtime_error("Monitoring config conflict for topic '" + ts.name + "'");
          }
          r.stats = &st;
        }
      }
    }

    // compression needs the deserialized image => typed subscription
    for (auto &[name, r]: routes_) {
      if (!r->compressed_bags.empty() && r->mode == "generic") {
        RCLCPP_WARN(get_logger(), "Topic '%s': image compression requires a typed subscription; "
                    "overriding mode 'generic' -> 'typed'.", name.c_str());
        r->mode = "typed";
      }
    }
  }

  // ---------------- rosbag2 open & topics ----------------

  void BagRecorderComponent::open_bag_and_create_topics(BagRuntime &br) const {
    rosbag2_storage::StorageOptions storage_options;

    storage_options.uri = br.spec.bag_uri + "/" + br.spec.name;
    if (br.spec.uri_with_datetime) {
      storage_options.uri += "_";
      storage_options.uri += now_string_local();
    }

    storage_options.storage_id = br.spec.storage_id;
    storage_options.snapshot_mode = false;
    storage_options.max_cache_size = br.spec.bag_cache_size;
    storage_options.max_bagfile_size = br.spec.max_bag_size;
    storage_options.storage_config_uri = br.spec.storage_config_uri;

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    br.writer.open(storage_options, converter_options);

    for (const auto &ts: br.spec.topics) {
      rosbag2_storage::TopicMetadata md;
      if (br.spec.compress_images && is_image_type(ts.type)) {
        md.name = ts.name + "/compressed";
        md.type = "sensor_msgs/msg/CompressedImage";
      } else {
        md.name = ts.name;
        md.type = ts.type;
      }
      md.serialization_format = "cdr";
      br.writer.create_topic(md);
    }

    RCLCPP_INFO(get_logger(),
                "Opened bag '%s' uri='%s' storage_id='%s' topics=%zu max_queue_bytes=%zu cache=%zu "
                "overflow_policy=%s compress_images=%s",
                br.spec.name.c_str(), storage_options.uri.c_str(), br.spec.storage_id.c_str(),
                br.spec.topics.size(), br.spec.max_queue_bytes, br.spec.bag_cache_size,
                policy_name(br.spec.overflow_policy),
                br.spec.compress_images ? br.spec.compression.format.c_str() : "off");

    if (br.spec.bag_cache_size > 0) {
      RCLCPP_WARN(get_logger(),
                  "Bag '%s': bag_cache_size>0 adds rosbag2's cache behind the recorder queue. That cache drops "
                  "messages silently when full (only reported when the bag is closed), so such drops are not shown "
                  "in the monitor. Prefer bag_cache_size: 0 and size max_queue_bytes instead.",
                  br.spec.name.c_str());
    }
    if (br.spec.overflow_policy == BagSpec::OverflowPolicy::BLOCK) {
      RCLCPP_WARN(get_logger(),
                  "Bag '%s': overflow_policy=block does NOT prevent data loss: a blocked callback lets the "
                  "subscription's QoS queue overflow (uncounted) and stalls every topic in its callback group. "
                  "Prefer drop_oldest with a large max_queue_bytes.", br.spec.name.c_str());
    }
  }

  // ---------------- subscriptions ----------------

  void BagRecorderComponent::create_subscriptions() {
    subs_.clear();
    subs_.reserve(routes_.size());

    for (auto &[topic, rp]: routes_) {
      TopicRoute &r = *rp;
      const auto qos = qos_from_string(r.qos, r.qos_depth);
      rclcpp::SubscriptionOptions options;
      options.callback_group = r.cb_group;

      const bool registered = type_registry_.contains(r.type);
      bool typed = registered && r.mode != "generic";
      if (r.mode == "typed" && !registered) {
        RCLCPP_WARN(get_logger(), "Topic '%s': type '%s' is not registered for typed mode; "
                    "falling back to a generic (serialized) subscription.", topic.c_str(), r.type.c_str());
      }
      if (!r.compressed_bags.empty() && !typed) {
        // cannot happen for sensor_msgs/msg/Image, which is always registered
        RCLCPP_ERROR(get_logger(), "Topic '%s': cannot compress without a typed subscription; "
                     "it will NOT be recorded in the compressing bags.", topic.c_str());
        r.compressed_bags.clear();
      }

      try {
        auto sub = typed
                     ? type_registry_.at(r.type)(r, qos, options)
                     : create_generic_route_subscription(r, qos, options);
        subs_.push_back(std::move(sub));
        RCLCPP_DEBUG(get_logger(), "Subscribed '%s' [%s] mode=%s qos=%s", topic.c_str(), r.type.c_str(),
                     typed ? "typed" : "generic", r.qos.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to create subscription for topic='%s' type='%s': %s",
                     topic.c_str(), r.type.c_str(), e.what());
      }
    }
  }

  rclcpp::SubscriptionBase::SharedPtr BagRecorderComponent::create_generic_route_subscription(
    TopicRoute &route, const rclcpp::QoS &qos, const rclcpp::SubscriptionOptions &options) {
    TopicRoute *r = &route;
    auto life = life_;
    // Generic subscription: callback receives serialized bytes already (goes through DDS, no intra-process).
    auto cb = [this, r, life](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      const CallbackScope scope(*life);
      if (!scope.ok()) return;
      try {
        on_rx(r->stats, now_ns_steady());
        const int64_t recv_ns = this->now().nanoseconds();
        // Zero-copy: the executor allocates a fresh SerializedMessage per take, so we can
        // keep it alive and alias its buffer instead of memcpy'ing it.
        PayloadPtr payload(msg, &msg->get_rcl_serialized_message());
        // can't extract header stamp without deserializing
        dispatch(r->raw_bags, r->topic, recv_ns, recv_ns, std::move(payload));
      } catch (const std::exception &e) {
        report_callback_error(*r, e.what());
      }
    };
    return this->create_generic_subscription(route.topic, route.type, qos, std::move(cb), options);
  }

  void BagRecorderComponent::report_callback_error(TopicRoute &route, const std::string &what) {
    if (should_warn(route.last_error_warn_ns, now_ns_steady(), 1.0)) {
      RCLCPP_ERROR(get_logger(), "Topic '%s': message not recorded: %s", route.topic.c_str(), what.c_str());
    }
  }

  // ---------------- queueing ----------------

  void BagRecorderComponent::dispatch(const std::vector<BagRuntime *> &bags,
                                      const std::string &bag_topic,
                                      int64_t send_ns,
                                      const int64_t recv_ns,
                                      PayloadPtr payload) {
    if (bags.empty()) return;
    // sane lower bound, 2020-01-01 UTC; anything below is an unset/garbage stamp
    static constexpr int64_t kMinSaneNs = 1577836800000000000LL;
    if (send_ns < kMinSaneNs) {
      send_ns = recv_ns; // fall back to receive time instead of writing 1970
    }
    auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag_msg->topic_name = bag_topic;
    set_bag_message_timestamps(*bag_msg, send_ns, recv_ns);
    bag_msg->serialized_data = std::move(payload);

    // the same (read-only) message is shared by all bags
    for (BagRuntime *br: bags) {
      enqueue(*br, bag_msg);
    }
  }

  void BagRecorderComponent::enqueue(BagRuntime &br, const BagMsgPtr &msg) {
    const BagSpec &spec = br.spec;
    const size_t msg_bytes = msg->serialized_data ? msg->serialized_data->buffer_length : 0;

    auto record_drop = [&br](const size_t bytes) {
      br.dropped_msgs.fetch_add(1, std::memory_order_relaxed);
      br.dropped_since_warn.fetch_add(1, std::memory_order_relaxed);
      br.dropped_bytes_since_warn.fetch_add(bytes, std::memory_order_relaxed);
    };

    bool accepted = true;
    bool block_warn = false;
    size_t snap_msgs = 0, snap_bytes = 0;
    {
      std::unique_lock<std::mutex> lk(br.mtx);
      if (br.stop) return;

      // pending = queued + currently being written by the writer thread
      auto fits = [&]() {
        return br.pending_msgs < spec.max_queue_size && br.pending_bytes + msg_bytes <= spec.max_queue_bytes;
      };

      if (spec.overflow_policy == BagSpec::OverflowPolicy::BLOCK) {
        // A message larger than the whole cap is let through once nothing else is pending,
        // instead of waiting forever.
        if (!fits() && br.pending_msgs > 0) {
          block_warn = spec.warn_on_overflow;
          snap_msgs = br.pending_msgs;
          snap_bytes = br.pending_bytes;
          ++br.blocked_producers;
          br.cv_not_full.wait(lk, [&]() { return br.stop || br.pending_msgs == 0 || fits(); });
          --br.blocked_producers;
          if (br.stop) return;
        }
      } else {
        // check first: a message that can never fit must not flush the queue
        const bool can_ever_fit = msg_bytes <= spec.max_queue_bytes && spec.max_queue_size > 0;
        if (can_ever_fit) {
          while (!fits() && !br.queue.empty()) {
            const auto &old = br.queue.front();
            const size_t old_bytes = old->serialized_data ? old->serialized_data->buffer_length : 0;
            br.queue.pop_front();
            --br.pending_msgs;
            br.pending_bytes -= old_bytes;
            record_drop(old_bytes);
          }
        }
        // still doesn't fit (or the batch being written occupies the budget): drop incoming
        if (!can_ever_fit || !fits()) {
          record_drop(msg_bytes);
          accepted = false;
        }
      }

      if (accepted) {
        br.queue.push_back(msg);
        ++br.pending_msgs;
        br.pending_bytes += msg_bytes;
        br.enqueued_msgs.fetch_add(1, std::memory_order_relaxed);
      }
      if (!block_warn) {
        snap_msgs = br.pending_msgs;
        snap_bytes = br.pending_bytes;
      }
    }
    if (accepted) br.cv_not_empty.notify_one();

    if (block_warn && should_warn(br.last_overflow_warn_ns, now_ns_steady(), spec.overflow_warn_period_sec)) {
      RCLCPP_WARN(get_logger(),
                  "[bag=%s] OVERFLOW(block): writer is behind; producer was blocked. pending=%zu msgs (%zu bytes), "
                  "incoming=%zu bytes topic=%s, limits: max_queue_size=%zu max_queue_bytes=%zu",
                  spec.name.c_str(), snap_msgs, snap_bytes, msg_bytes, msg->topic_name.c_str(),
                  spec.max_queue_size, spec.max_queue_bytes);
    }

    // rate-limited aggregated drop warning
    if (spec.warn_on_overflow && br.dropped_since_warn.load(std::memory_order_relaxed) > 0 &&
        should_warn(br.last_overflow_warn_ns, now_ns_steady(), spec.overflow_warn_period_sec)) {
      const auto dm = br.dropped_since_warn.exchange(0, std::memory_order_relaxed);
      const auto db = br.dropped_bytes_since_warn.exchange(0, std::memory_order_relaxed);
      RCLCPP_WARN(get_logger(),
                  "[bag=%s] OVERFLOW(drop_oldest): dropped=%llu msgs (%llu bytes) since last warning | "
                  "pending=%zu msgs (%zu bytes) | limits: max_queue_size=%zu max_queue_bytes=%zu",
                  spec.name.c_str(), static_cast<unsigned long long>(dm), static_cast<unsigned long long>(db),
                  snap_msgs, snap_bytes, spec.max_queue_size, spec.max_queue_bytes);
    }
  }

  // ---------------- writer thread ----------------

  void BagRecorderComponent::writer_loop(BagRuntime &br) const {
    std::deque<BagMsgPtr> batch;
    while (true) {
      {
        std::unique_lock<std::mutex> lk(br.mtx);
        br.cv_not_empty.wait(lk, [&]() { return br.stop || !br.queue.empty(); });
        if (br.queue.empty()) break; // stop requested and everything is drained
        batch.swap(br.queue); // O(1)
      }

      for (auto &bag_msg: batch) {
        const size_t bytes = bag_msg->serialized_data ? bag_msg->serialized_data->buffer_length : 0;
        try {
          br.writer.write(bag_msg);
          br.written_msgs.fetch_add(1, std::memory_order_relaxed);
        } catch (const std::exception &e) {
          // e.g. disk full: never let an exception escape a std::thread (std::terminate kills the container)
          br.write_errors.fetch_add(1, std::memory_order_relaxed);
          if (should_warn(br.last_error_warn_ns, now_ns_steady(), 1.0)) {
            RCLCPP_ERROR(get_logger(), "[bag=%s] write failed on topic '%s': %s", br.spec.name.c_str(),
                         bag_msg->topic_name.c_str(), e.what());
          }
        }
        bag_msg.reset();

        // release budget per message, so blocked producers resume as soon as there is room
        bool wake = false;
        {
          std::lock_guard<std::mutex> lk(br.mtx);
          --br.pending_msgs;
          br.pending_bytes -= bytes;
          wake = br.blocked_producers > 0;
        }
        if (wake) br.cv_not_full.notify_all();
      }
      batch.clear();
    }

    RCLCPP_INFO(get_logger(), "Draining complete. Closing bag '%s' (written=%llu dropped=%llu errors=%llu)...",
                br.spec.name.c_str(),
                static_cast<unsigned long long>(br.written_msgs.load()),
                static_cast<unsigned long long>(br.dropped_msgs.load()),
                static_cast<unsigned long long>(br.write_errors.load()));
    try {
      br.writer.close();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "[bag=%s] close failed: %s", br.spec.name.c_str(), e.what());
    }
    RCLCPP_INFO(get_logger(), "Writer thread stopped for bag '%s'", br.spec.name.c_str());
  }

  //----------------- monitoring --------------------

  void BagRecorderComponent::on_rx(TopicStats *st, const int64_t recv_ns_steady) const {
    if (st == nullptr || !monitor_cfg_.enabled) return;

    st->count.fetch_add(1, std::memory_order_relaxed);

    // Each topic is served by exactly one subscription in a MutuallyExclusive group,
    // so the read-modify-write below never runs concurrently for the same topic.
    const int64_t prev = st->last_rx_ns.exchange(recv_ns_steady, std::memory_order_relaxed);
    if (prev == 0) return;

    const double dt = static_cast<double>(recv_ns_steady - prev) * 1e-9;
    if (dt <= 0.0) return;

    const double inst_hz = 1.0 / dt;
    const double tau = monitor_cfg_.ewma_tau_sec;
    const double alpha = (tau > 1e-6) ? (1.0 - std::exp(-dt / tau)) : 1.0;

    const double prev_ewma = st->ewma_hz.load(std::memory_order_relaxed);
    const double next_ewma = (prev_ewma <= 0.0) ? inst_hz : (prev_ewma + alpha * (inst_hz - prev_ewma));
    st->ewma_hz.store(next_ewma, std::memory_order_relaxed);

    if (st->warn_below_hz > 0.0) {
      if (next_ewma < st->warn_below_hz) {
        int64_t zero = 0;
        (void) st->below_since_ns.compare_exchange_strong(zero, recv_ns_steady, std::memory_order_relaxed);
      } else {
        st->below_since_ns.store(0, std::memory_order_relaxed);
      }
    }
  }

  void BagRecorderComponent::monitor_timer_cb() {
    const bool color = monitor_cfg_.use_ansi_color;
    if (monitor_cfg_.clear_screen) {
      std::cout << "\033[2J\033[H" << std::flush; // clear terminal + cursor to top-left
    }
    const int64_t now_ns = now_ns_steady();

    // ===== 1. topic statistics (received) =====
    RCLCPP_INFO(get_logger(), "===== BagRecorder Monitor (EWMA tau=%.2fs) =====", monitor_cfg_.ewma_tau_sec);
    RCLCPP_INFO(get_logger(), "%-40s  %8s  %8s  %8s  %s", "topic", "ewmaHz", "warn<", "count", "rate");

    for (auto &[topic, st]: topic_stats_) {
      // A topic that stopped publishing keeps its last EWMA forever; decay it by the silence.
      double ewma = st.ewma_hz.load(std::memory_order_relaxed);
      const int64_t last = st.last_rx_ns.load(std::memory_order_relaxed);
      if (last != 0 && ewma > 0.0) {
        const double silence = static_cast<double>(now_ns - last) * 1e-9;
        if (silence > 0.0) ewma = std::min(ewma, 1.0 / silence);
      }
      const uint64_t cnt = st.count.load(std::memory_order_relaxed);
      const double warn_below = st.warn_below_hz;

      bool warn = false;
      if (warn_below > 0.0) {
        if (ewma < warn_below && last != 0 &&
            static_cast<double>(now_ns - last) * 1e-9 >= monitor_cfg_.warn_grace_sec) {
          warn = true; // silent topic
        } else if (const int64_t below_since = st.below_since_ns.load(std::memory_order_relaxed);
          below_since != 0 && static_cast<double>(now_ns - below_since) * 1e-9 >= monitor_cfg_.warn_grace_sec) {
          warn = true;
        }
      }

      const double ref = (st.expected_hz > 0.0) ? st.expected_hz : ((warn_below > 0.0) ? warn_below : 1.0);
      char line[256];
      std::snprintf(line, sizeof(line), "%-40s  %8.2f  %8.2f  %8llu  [%s]",
                    topic.c_str(), ewma, warn_below, static_cast<unsigned long long>(cnt),
                    bar(ewma, ref, 20).c_str());

      if (warn || (warn_below > 0.0 && cnt == 0)) {
        RCLCPP_WARN(get_logger(), "%s", maybe_red(line, color).c_str());
      } else if (warn_below > 0.0 && ewma < warn_below) {
        RCLCPP_INFO(get_logger(), "%s", maybe_yellow(line, color).c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", line);
      }
    }

    // ===== 2. bag I/O statistics =====
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), "===== Bag I/O Statistics =====");
    RCLCPP_INFO(get_logger(), "%-20s  %10s  %10s  %10s  %10s  %10s  %8s",
                "Bag Name", "Enqueued", "Written", "Dropped", "Pending", "PendingMB", "Errors");

    for (const auto &br: bags_) {
      size_t pending_msgs = 0, pending_bytes = 0;
      {
        std::lock_guard<std::mutex> lk(br->mtx);
        pending_msgs = br->pending_msgs;
        pending_bytes = br->pending_bytes;
      }
      const auto enqueued = br->enqueued_msgs.load(std::memory_order_relaxed);
      const auto written = br->written_msgs.load(std::memory_order_relaxed);
      const auto dropped = br->dropped_msgs.load(std::memory_order_relaxed);
      const auto errors = br->write_errors.load(std::memory_order_relaxed);

      char bag_line[256];
      std::snprintf(bag_line, sizeof(bag_line), "%-20s  %10llu  %10llu  %10llu  %10zu  %10.1f  %8llu",
                    br->spec.name.c_str(),
                    static_cast<unsigned long long>(enqueued),
                    static_cast<unsigned long long>(written),
                    static_cast<unsigned long long>(dropped),
                    pending_msgs,
                    static_cast<double>(pending_bytes) / (1024.0 * 1024.0),
                    static_cast<unsigned long long>(errors));

      if (dropped > 0 || errors > 0) {
        RCLCPP_WARN(get_logger(), "%s", maybe_red(bag_line, color).c_str());
      } else {
        RCLCPP_INFO(get_logger(), "%s", bag_line);
      }
    }
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(lsy_ros_data_utils::rosbag::BagRecorderComponent)
