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

#ifndef LSY_ROS_DATA_UTILS_BAG_RECORDER_COMPONENT_HPP
#define LSY_ROS_DATA_UTILS_BAG_RECORDER_COMPONENT_HPP
// general utils
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

// ros utils
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/topic_metadata.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include "utils.hpp"
#include "msg_converter.hpp"

namespace lsy_ros_data_utils::rosbag {
  struct TopicSpec {
    std::string name;
    std::string type;
    std::string qos{"default"}; // normalized: "sensor_data" or "default"
    size_t qos_depth{0}; // 0 => use the profile's default depth
    std::string mode{"auto"}; // "auto" | "typed" (intra-process capable) | "generic" (serialized)
    rclcpp::CallbackGroup::SharedPtr cb_group;
    bool monitor_enabled{false};
    double expected_hz{0.0};
    double warn_below_hz{0.0};
  };

  struct BagSpec {
    enum class OverflowPolicy {
      DROP_OLDEST,
      BLOCK
    };

    std::string name;
    std::string bag_uri;
    bool uri_with_datetime{true};
    std::string storage_id{"mcap"};
    std::string storage_config_uri;

    // queue limits (queued + currently being written)
    size_t max_queue_size{20000}; // cap by message count
    size_t max_queue_bytes{1ull << 30}; // cap by bytes, 1 GiB default
    size_t bag_cache_size{0}; // rosbag2 internal cache; 0 (default) = write-through, our queue is the only buffer
    size_t max_bag_size{50ull << 30}; // split size, 50 GiB default (0 = no splitting)

    OverflowPolicy overflow_policy{OverflowPolicy::DROP_OLDEST};
    bool warn_on_overflow{true};
    double overflow_warn_period_sec{1.0}; // rate-limit warnings

    // image compression: applies to sensor_msgs/msg/Image topics of this bag,
    // which are then recorded as sensor_msgs/msg/CompressedImage on "<topic>/compressed"
    bool compress_images{false};
    ImageCompressionCfg compression;

    std::vector<TopicSpec> topics;
  };

  class BagRecorderComponent final : public rclcpp::Node {
  public:
    explicit BagRecorderComponent(const rclcpp::NodeOptions &node_options);

    ~BagRecorderComponent() override;

  private:
    using BagMsgPtr = std::shared_ptr<rosbag2_storage::SerializedBagMessage>;
    using PayloadPtr = std::shared_ptr<rcutils_uint8_array_t>;

    struct MonitorCfg {
      bool enabled{false};
      double print_hz{1.0};
      double ewma_tau_sec{2.0};
      double warn_grace_sec{2.0};
      bool use_ansi_color{true};
      bool clear_screen{true};
    };

    struct TopicStats {
      double expected_hz{0.0};
      double warn_below_hz{0.0};

      std::atomic<uint64_t> count{0};
      std::atomic<int64_t> last_rx_ns{0}; // steady clock ns
      std::atomic<double> ewma_hz{0.0};
      std::atomic<int64_t> below_since_ns{0}; // steady clock ns, 0 => not below
    };

    // bag runtime
    struct BagRuntime {
      BagSpec spec;
      rosbag2_cpp::Writer writer;

      // ---- guarded by mtx ----
      std::mutex mtx;
      std::condition_variable cv_not_empty; // writer waits on this
      std::condition_variable cv_not_full; // BLOCK-mode producers wait on this
      std::deque<BagMsgPtr> queue;
      size_t pending_msgs{0}; // queued + currently being written
      size_t pending_bytes{0}; // queued + currently being written
      size_t blocked_producers{0};
      bool stop{false};

      std::thread thread;

      // ---- monotonic statistics ----
      std::atomic<uint64_t> enqueued_msgs{0};
      std::atomic<uint64_t> written_msgs{0};
      std::atomic<uint64_t> dropped_msgs{0};
      std::atomic<uint64_t> write_errors{0};

      // ---- rate-limited warning state ----
      std::atomic<uint64_t> dropped_since_warn{0};
      std::atomic<uint64_t> dropped_bytes_since_warn{0};
      std::atomic<int64_t> last_overflow_warn_ns{0}; // steady clock ns
      std::atomic<int64_t> last_error_warn_ns{0}; // steady clock ns
    };

    // One per unique subscribed topic. Resolved once at startup, so the hot path
    // does no map lookups. Stored behind unique_ptr => stable addresses.
    struct TopicRoute {
      std::string topic;
      std::string type;
      std::string compressed_topic; // topic name used in bags that compress this image topic
      std::string qos{"default"};
      size_t qos_depth{0};
      std::string mode{"auto"};
      rclcpp::CallbackGroup::SharedPtr cb_group;

      TopicStats *stats{nullptr}; // nullptr => not monitored
      std::vector<BagRuntime *> raw_bags; // bags recording the message as-is
      std::vector<BagRuntime *> compressed_bags; // bags recording a compressed image
      ImageCompressionCfg compression;

      std::atomic<int64_t> last_error_warn_ns{0};
    };

    // Shared with every callback (captured by value), so a callback that races with
    // destruction can still safely check whether the node is going away.
    struct Lifetime {
      std::atomic<bool> shutting_down{false};
      std::atomic<int> active_callbacks{0};
    };

    class CallbackScope {
    public:
      explicit CallbackScope(Lifetime &life) : life_(life) {
        // increment first, then check: pairs with request_stop() (set flag, then wait for 0)
        life_.active_callbacks.fetch_add(1);
        ok_ = !life_.shutting_down.load();
      }

      ~CallbackScope() { life_.active_callbacks.fetch_sub(1); }

      CallbackScope(const CallbackScope &) = delete;

      CallbackScope &operator=(const CallbackScope &) = delete;

      [[nodiscard]] bool ok() const { return ok_; }

    private:
      Lifetime &life_;
      bool ok_{false};
    };

    // ---------- setup ----------
    void load_config(const std::string &yaml_path);

    void build_routes();

    void open_bag_and_create_topics(BagRuntime &br) const;

    void create_subscriptions();

    rclcpp::SubscriptionBase::SharedPtr create_generic_route_subscription(
      TopicRoute &route, const rclcpp::QoS &qos, const rclcpp::SubscriptionOptions &options);

    void init_type_registry();

    template<typename MsgT>
    void register_type(const std::string &type_name);

    // ---------- hot path ----------
    template<typename MsgT>
    void handle_typed(TopicRoute &route, const MsgT &msg);

    void dispatch(const std::vector<BagRuntime *> &bags,
                  const std::string &bag_topic,
                  int64_t send_ns,
                  int64_t recv_ns,
                  PayloadPtr payload);

    void enqueue(BagRuntime &br, const BagMsgPtr &msg);

    void report_callback_error(TopicRoute &route, const std::string &what);

    static PayloadPtr to_payload(rclcpp::SerializedMessage &&serialized);

    static void set_bag_message_timestamps(
      rosbag2_storage::SerializedBagMessage &m, int64_t send_ns, int64_t recv_ns);

    // ---------- writer ----------
    void writer_loop(BagRuntime &br) const;

    void request_stop();

    // ---------- monitoring ----------
    void on_rx(TopicStats *st, int64_t recv_ns_steady) const;

    void monitor_timer_cb();

    // ---------- variables ----------
    std::string cfg_file_;
    std::shared_ptr<Lifetime> life_ = std::make_shared<Lifetime>();
    rclcpp::Context::SharedPtr context_;
    std::optional<rclcpp::OnShutdownCallbackHandle> shutdown_cb_handle_;

    std::vector<std::unique_ptr<BagRuntime> > bags_;
    std::map<std::string, std::unique_ptr<TopicRoute> > routes_;
    std::vector<rclcpp::CallbackGroup::SharedPtr> cb_groups_;

    // registry: "sensor_msgs/msg/Image" -> factory that creates a typed (intra-process capable) sub
    using SubFactory = std::function<rclcpp::SubscriptionBase::SharedPtr(
      TopicRoute &route,
      const rclcpp::QoS &qos,
      const rclcpp::SubscriptionOptions &options)>;
    std::unordered_map<std::string, SubFactory> type_registry_;

    std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;

    // monitoring (std::map => sorted output, stable element addresses)
    MonitorCfg monitor_cfg_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    std::map<std::string, TopicStats> topic_stats_;
  };

  // ============ template implementation (must be in header) ============

  template<typename MsgT>
  void BagRecorderComponent::register_type(const std::string &type_name) {
    type_registry_[type_name] =
        [this](TopicRoute &route,
               const rclcpp::QoS &qos,
               const rclcpp::SubscriptionOptions &options) -> rclcpp::SubscriptionBase::SharedPtr {
          TopicRoute *r = &route;
          auto life = life_;
          // ConstSharedPtr callback: with intra-process comms the message is delivered without a copy.
          auto cb = [this, r, life](typename MsgT::ConstSharedPtr msg) {
            const CallbackScope scope(*life);
            if (!scope.ok()) return;
            try {
              this->handle_typed<MsgT>(*r, *msg);
            } catch (const std::exception &e) {
              this->report_callback_error(*r, e.what());
            }
          };
          return this->create_subscription<MsgT>(route.topic, qos, std::move(cb), options);
        };
  }

  template<typename MsgT>
  void BagRecorderComponent::handle_typed(TopicRoute &route, const MsgT &msg) {
    on_rx(route.stats, now_ns_steady());

    const int64_t recv_ns = this->now().nanoseconds();
    const int64_t send_ns = get_message_timestamp_ns(msg, recv_ns);

    if (!route.raw_bags.empty()) {
      static const rclcpp::Serialization<MsgT> ser;
      rclcpp::SerializedMessage serialized;
      ser.serialize_message(&msg, &serialized);
      dispatch(route.raw_bags, route.topic, send_ns, recv_ns, to_payload(std::move(serialized)));
    }

    if constexpr (std::is_same_v<MsgT, sensor_msgs::msg::Image>) {
      if (!route.compressed_bags.empty()) {
        std::string err;
        const auto compressed = to_compressed(msg, route.compression, &err);
        if (!compressed) {
          report_callback_error(route, "image compression failed: " + err);
          return;
        }
        static const rclcpp::Serialization<sensor_msgs::msg::CompressedImage> ser;
        rclcpp::SerializedMessage serialized;
        ser.serialize_message(&*compressed, &serialized);
        dispatch(route.compressed_bags, route.compressed_topic, send_ns, recv_ns,
                 to_payload(std::move(serialized)));
      }
    }
  }
}

#endif //LSY_ROS_DATA_UTILS_BAG_RECORDER_COMPONENT_HPP
