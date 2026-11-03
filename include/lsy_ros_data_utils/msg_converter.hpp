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
// Created by haoming on 2/8/26.
//

#ifndef LSY_ROS_DATA_UTILS_MSG_CONVERTER_HPP
#define LSY_ROS_DATA_UTILS_MSG_CONVERTER_HPP

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>
#include <vector>

namespace lsy_ros_data_utils {
  struct ImageCompressionCfg {
    std::string format{"png"}; // "png" or "jpeg" (normalized)
    int jpeg_quality{95}; // [0..100]
    int png_level{3}; // [0..9], higher = smaller, slower

    bool operator==(const ImageCompressionCfg &) const = default;
  };

  // Returns "png" / "jpeg", or an empty string if unsupported.
  inline std::string normalize_compression_format(std::string fmt) {
    std::transform(fmt.begin(), fmt.end(), fmt.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (fmt == "jpg") fmt = "jpeg";
    if (fmt == "png" || fmt == "jpeg") return fmt;
    return {};
  }

  struct EncodingInfo {
    int cv_type{0};
    int bit_depth{8};
    bool color{false};
    int to_bgr{-1}; // cv::ColorConversionCodes, -1 => already BGR / mono
  };

  inline std::optional<EncodingInfo> encoding_info(const std::string &enc) {
    if (enc == "bgr8") return EncodingInfo{CV_8UC3, 8, true, -1};
    if (enc == "rgb8") return EncodingInfo{CV_8UC3, 8, true, cv::COLOR_RGB2BGR};
    if (enc == "bgra8") return EncodingInfo{CV_8UC4, 8, true, cv::COLOR_BGRA2BGR};
    if (enc == "rgba8") return EncodingInfo{CV_8UC4, 8, true, cv::COLOR_RGBA2BGR};
    if (enc == "mono8" || enc == "8UC1") return EncodingInfo{CV_8UC1, 8, false, -1};
    if (enc == "mono16" || enc == "16UC1") return EncodingInfo{CV_16UC1, 16, false, -1};
    return std::nullopt;
  }

  // Raw Image -> CompressedImage, following the compressed_image_transport conventions
  // (color images are stored as BGR, format string "<enc>; <png|jpeg> compressed <target>"),
  // so the result can be decoded by image_transport / Foxglove.
  // Returns std::nullopt (and fills *error) instead of throwing.
  inline std::optional<sensor_msgs::msg::CompressedImage> to_compressed(
    const sensor_msgs::msg::Image &in,
    const ImageCompressionCfg &cfg,
    std::string *error = nullptr) {
    auto fail = [&](const std::string &msg) -> std::optional<sensor_msgs::msg::CompressedImage> {
      if (error) *error = msg;
      return std::nullopt;
    };

    const std::string fmt = normalize_compression_format(cfg.format);
    if (fmt.empty()) return fail("unsupported compression format '" + cfg.format + "'");

    const auto info = encoding_info(in.encoding);
    if (!info) return fail("unsupported image encoding '" + in.encoding + "'");
    if (fmt == "jpeg" && info->bit_depth != 8) {
      return fail("JPEG cannot store " + std::to_string(info->bit_depth) + "-bit image '" + in.encoding + "'");
    }

    const size_t min_step = static_cast<size_t>(in.width) * CV_ELEM_SIZE(info->cv_type);
    if (in.width == 0 || in.height == 0 || in.step < min_step ||
        in.data.size() < static_cast<size_t>(in.step) * in.height) {
      return fail("inconsistent image size (w=" + std::to_string(in.width) + " h=" + std::to_string(in.height) +
                  " step=" + std::to_string(in.step) + " data=" + std::to_string(in.data.size()) + ")");
    }

    // Wrap input buffer without copying (OpenCV API is non-const, we never modify it).
    const cv::Mat src(static_cast<int>(in.height), static_cast<int>(in.width), info->cv_type,
                      const_cast<uint8_t *>(in.data.data()), static_cast<size_t>(in.step));

    thread_local cv::Mat converted;
    const cv::Mat *to_encode = &src;
    if (info->to_bgr >= 0) {
      cv::cvtColor(src, converted, info->to_bgr);
      to_encode = &converted;
    }

    std::vector<int> params;
    std::string ext;
    if (fmt == "png") {
      ext = ".png";
      params = {cv::IMWRITE_PNG_COMPRESSION, std::clamp(cfg.png_level, 0, 9)};
    } else {
      ext = ".jpg";
      params = {cv::IMWRITE_JPEG_QUALITY, std::clamp(cfg.jpeg_quality, 0, 100)};
    }

    sensor_msgs::msg::CompressedImage out;
    out.header = in.header;
    out.format = in.encoding + "; " + fmt + " compressed ";
    if (info->color) out.format += "bgr" + std::to_string(info->bit_depth);

    try {
      // encode straight into the message buffer (same vector type => no extra copy)
      if (!cv::imencode(ext, *to_encode, out.data, params)) return fail("cv::imencode returned false");
    } catch (const cv::Exception &e) {
      return fail(std::string("cv::imencode threw: ") + e.what());
    }
    return out;
  }
}

#endif //LSY_ROS_DATA_UTILS_MSG_CONVERTER_HPP
