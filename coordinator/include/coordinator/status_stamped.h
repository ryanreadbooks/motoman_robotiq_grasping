#pragma once

#include <string>

#include <geometry_msgs/PoseStamped.h>

#include "configor/json.hpp"

class StatusStamped {
 public:
  friend class StatusStampedHolder;

  StatusStamped() = default;

  StatusStamped(const std::string& name, const geometry_msgs::PoseStamped& pose_status);

  StatusStamped(const std::string& name, const geometry_msgs::Pose& pose_status);

  // 将StatusStamped对象简单地序列化成字符串
  std::string as_string() const;

  std::string to_stdstring() const;

  // 将字符串反序列化会StatusStamped对象
  static bool from_string(const std::string& s, StatusStamped& out);

  static bool from_buffer(char* s, size_t maxlen, size_t idx, StatusStamped& out, size_t& new_idx);

  geometry_msgs::PoseStamped pose_stamped() {
    return pose_status_;
  }

  geometry_msgs::Pose pose() {
    geometry_msgs::Pose pose;
    pose.position.x = pose_status_.pose.position.x;
    pose.position.y = pose_status_.pose.position.y;
    pose.position.z = pose_status_.pose.position.z;
    pose.orientation.x = pose_status_.pose.orientation.x;
    pose.orientation.y = pose_status_.pose.orientation.y;
    pose.orientation.z = pose_status_.pose.orientation.z;
    pose.orientation.w = pose_status_.pose.orientation.w;
    return pose;
  }

  std::string get_name() const {
    return name_;
  }

  uint64_t get_timestamp() const {
    return timestamp_;
  }

 private:
  uint64_t timestamp_;
  std::string name_;
  geometry_msgs::PoseStamped pose_status_;
};

class StatusStampedHolder {
 public:
  void add(const std::string& name, const StatusStamped& status);

  bool has(const std::string& name) const {
    return datas.count(name) != 0;
  }

  void save_to(const std::string& filename) const;

  bool load_from(const std::string& filename);

  void save_to_json(const std::string& filename) const;

  bool load_from_json(const std::string& filename);

 public:
  // 为了方便 直接公有...
  std::unordered_map<std::string, StatusStamped> datas;
};