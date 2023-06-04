#include <fstream>
#include <sstream>

#include "configor/json.hpp"  // for json
#include "coordinator/status_stamped.h"

using namespace configor;

const char* crlf = "\r\n";

// 将8个字节的内容写入stringstream流中
static void eight_bytes_into_stream(std::stringstream& ss, char* buf) {
  for (int i = 0; i < 8; i++) {
    ss << char(*(buf + i));
  }
}

StatusStamped::StatusStamped(const std::string& name, const geometry_msgs::PoseStamped& pose_status)
    : name_(name), pose_status_(pose_status) {
  timestamp_ = time(nullptr);
}

StatusStamped::StatusStamped(const std::string& name, const geometry_msgs::Pose& pose_status) : name_(name) {
  pose_status_.pose.position.x = pose_status.position.x;
  pose_status_.pose.position.y = pose_status.position.y;
  pose_status_.pose.position.z = pose_status.position.z;
  pose_status_.pose.orientation.x = pose_status.orientation.x;
  pose_status_.pose.orientation.y = pose_status.orientation.y;
  pose_status_.pose.orientation.z = pose_status.orientation.z;
  pose_status_.pose.orientation.w = pose_status.orientation.w;
}

std::string StatusStamped::as_string() const {
  std::stringstream ss;

  // 小端
  // 0xffee开头
  ss << char(0xff) << char(0xee);
  // 时间戳 8 bytes
  // 小端存储
  uint64_t now = time(nullptr);
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&now));

  size_t name_len = name_.size();
  // 定长8字节放名字的长度
  char* plen = reinterpret_cast<char*>(&name_len);
  eight_bytes_into_stream(ss, plen);

  // 接着开始放名字
  ss << name_;

  // 接着开始放pose_status_
  auto x = pose_status_.pose.position.x;
  auto y = pose_status_.pose.position.y;
  auto z = pose_status_.pose.position.z;
  auto qx = pose_status_.pose.orientation.x;
  auto qy = pose_status_.pose.orientation.y;
  auto qz = pose_status_.pose.orientation.z;
  auto qw = pose_status_.pose.orientation.w;

  // crlf结尾
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&x));
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&y));
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&z));
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&qx));
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&qy));
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&qz));
  eight_bytes_into_stream(ss, reinterpret_cast<char*>(&qw));

  ss << crlf;

  return ss.str();
}

// 反序列化
bool StatusStamped::from_string(const std::string& s, StatusStamped& out) {
  size_t maxlen = s.size();
  if (maxlen == 0 || maxlen < 2) {
    return false;
  }

  // 0xffee开头
  size_t idx = 0;

  if (char(s[idx]) != char(0xff) || char(s[idx + 1]) != char(0xee)) {
    return false;
  }

  idx += 2;
  if (idx > maxlen || idx + 8 > maxlen) {
    return false;
  }
  // 后序8个字节是时间戳
  uint64_t timestamp = *(reinterpret_cast<uint64_t*>(const_cast<char*>(s.c_str() + idx)));

  idx += 8;
  if (idx > maxlen || idx + 8 > maxlen) {
    return false;
  }
  // 后面8个是name的长度
  uint64_t namelen = *(reinterpret_cast<uint64_t*>(const_cast<char*>(s.c_str() + idx)));
  idx += 8;
  if (idx > maxlen || idx + namelen > maxlen) {
    return false;
  }
  // 接着开始读namelen个字节
  std::string name(s.c_str() + idx, s.c_str() + idx + namelen);

  idx += namelen;
  if (idx > maxlen || idx + 56 > maxlen) {
    return false;
  }

  // 连续7个8字节
  double x = *(reinterpret_cast<double*>(const_cast<char*>(s.c_str() + idx)));
  double y = *(reinterpret_cast<double*>(const_cast<char*>(s.c_str() + idx + 8)));
  double z = *(reinterpret_cast<double*>(const_cast<char*>(s.c_str() + idx + 16)));
  double qx = *(reinterpret_cast<double*>(const_cast<char*>(s.c_str() + idx + 24)));
  double qy = *(reinterpret_cast<double*>(const_cast<char*>(s.c_str() + idx + 32)));
  double qz = *(reinterpret_cast<double*>(const_cast<char*>(s.c_str() + idx + 40)));
  double qw = *(reinterpret_cast<double*>(const_cast<char*>(s.c_str() + idx + 48)));

  idx += 56;

  if (idx > maxlen || idx + 2 > maxlen) {
    return false;
  }

  if (s[idx] != '\r' || s[idx + 1] != '\n') {
    return false;
  }

  idx += 2;

  out.timestamp_ = timestamp;
  out.name_ = name;
  geometry_msgs::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = z;
  p.pose.position.z = y;
  p.pose.orientation.x = qx;
  p.pose.orientation.y = qw;
  p.pose.orientation.z = qz;
  p.pose.orientation.w = qw;

  out.pose_status_ = p;

  return true;
}

bool StatusStamped::from_buffer(char* s, size_t maxlen, size_t idx, StatusStamped& out, size_t& new_idx) {
  if (maxlen == 0 || maxlen < 2) {
    return false;
  }

  // 0xffee开头

  if (char(s[idx]) != char(0xff) || char(s[idx + 1]) != char(0xee)) {
    return false;
  }

  idx += 2;
  if (idx > maxlen || idx + 8 > maxlen) {
    return false;
  }
  // 后序8个字节是时间戳
  uint64_t timestamp = *(reinterpret_cast<uint64_t*>(const_cast<char*>(s + idx)));

  idx += 8;
  if (idx > maxlen || idx + 8 > maxlen) {
    return false;
  }
  // 后面8个是name的长度
  uint64_t namelen = *(reinterpret_cast<uint64_t*>(const_cast<char*>(s + idx)));
  idx += 8;
  if (idx > maxlen || idx + namelen > maxlen) {
    return false;
  }
  // 接着开始读namelen个字节
  std::string name(s + idx, s + idx + namelen);

  idx += namelen;
  if (idx > maxlen || idx + 56 > maxlen) {
    return false;
  }

  // 连续7个8字节
  double x = *(reinterpret_cast<double*>(const_cast<char*>(s + idx)));
  double y = *(reinterpret_cast<double*>(const_cast<char*>(s + idx + 8)));
  double z = *(reinterpret_cast<double*>(const_cast<char*>(s + idx + 16)));
  double qx = *(reinterpret_cast<double*>(const_cast<char*>(s + idx + 24)));
  double qy = *(reinterpret_cast<double*>(const_cast<char*>(s + idx + 32)));
  double qz = *(reinterpret_cast<double*>(const_cast<char*>(s + idx + 40)));
  double qw = *(reinterpret_cast<double*>(const_cast<char*>(s + idx + 48)));

  idx += 56;

  if (idx > maxlen || idx + 2 > maxlen) {
    return false;
  }

  if (s[idx] != '\r' || s[idx + 1] != '\n') {
    return false;
  }

  idx += 2;

  out.timestamp_ = timestamp;
  out.name_ = name;
  geometry_msgs::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.position.z = z;
  p.pose.orientation.x = qx;
  p.pose.orientation.y = qw;
  p.pose.orientation.z = qz;
  p.pose.orientation.w = qw;

  out.pose_status_ = p;

  new_idx = idx;

  return true;
}

void StatusStampedHolder::add(const std::string& name, const StatusStamped& status) {
  datas[name] = status;
}

void StatusStampedHolder::save_to(const std::string& filename) const {
  std::stringstream ss;
  for (auto it = datas.begin(); it != datas.end(); it++) {
    ss << it->second.as_string();
  }

  std::ofstream ofs(filename);
  if (!ofs) {
    std::cout << "can not construct ofstream from " << filename << std::endl;
    return;
  }
  ofs << ss.str();
  ofs.close();
}

bool StatusStampedHolder::load_from(const std::string& filename) {
  // TODO 不要这样写！！！
  std::ofstream ofs(filename, std::ios::app);
  ofs.close();

  std::ifstream ifs(filename);
  if (!ifs) {
    // 不存在则创建文件
    std::cout << "can not construct ifstream from " << filename << std::endl;
    return false;
  }

  ifs.seekg(0, std::ios::end);
  size_t filelen = ifs.tellg();
  ifs.seekg(0, std::ios::beg);

  char* buf = new char[filelen];
  ifs.read(buf, filelen);

  size_t idx = 0;
  size_t num = 0;
  while (true) {
    StatusStamped sta;
    bool ret = StatusStamped::from_buffer(buf, filelen, idx, sta, idx);
    if (!ret) {
      break;
    }
    num++;
    datas.emplace(sta.name_, sta);
  }

  std::cout << "successfully read " << num << " statusStamped records\n";

  delete[] buf;
  ifs.close();

  return true;
}

void StatusStampedHolder::save_to_json(const std::string& filename) const {
  std::ofstream ofs(filename);
  if (!ofs) {
    std::cout << "can not construct ofstream from " << filename << std::endl;
    return;
  }
  // generate json format content
  std::vector<json> items;
  for (auto it = datas.begin(); it != datas.end(); it++) {
    items.emplace_back(json{
        {"name", it->first},
        {"timestamp", it->second.timestamp_},
        {"x", it->second.pose_status_.pose.position.x},
        {"y", it->second.pose_status_.pose.position.y},
        {"z", it->second.pose_status_.pose.position.z},
        {"qx", it->second.pose_status_.pose.orientation.x},
        {"qy", it->second.pose_status_.pose.orientation.y},
        {"qz", it->second.pose_status_.pose.orientation.z},
        {"qw", it->second.pose_status_.pose.orientation.w},
    });
  }
  json out(items);
  ofs << json::wrap(items);
  ofs.close();
}

bool StatusStampedHolder::load_from_json(const std::string& filename) {
  try {
    std::FILE* f = std::fopen(filename.c_str(), "r");
    if (f == nullptr) {
      throw std::runtime_error("can not open file");
    }
    json j = json::parse(f);
    for (auto it = j.begin(); it != j.end(); it++) {
      // std::cout << it->operator[]("name") << std::endl;
      StatusStamped status;
      status.name_ = it->operator[]("name").as_string();
      status.timestamp_ = it->operator[]("timestamp").as_integer();
      status.pose_status_.pose.position.x = it->operator[]("x").as_float();
      status.pose_status_.pose.position.y = it->operator[]("y").as_float();
      status.pose_status_.pose.position.z = it->operator[]("z").as_float();
      status.pose_status_.pose.orientation.x = it->operator[]("qx").as_float();
      status.pose_status_.pose.orientation.y = it->operator[]("qw").as_float();
      status.pose_status_.pose.orientation.z = it->operator[]("qz").as_float();
      status.pose_status_.pose.orientation.w = it->operator[]("qw").as_float();
      datas.emplace(status.name_, status);
    }
  } catch (std::exception& ex) {
    std::cout << "can not load " << filename << ": " << ex.what() << std::endl;
    return false;
  }
  return true;
}

std::string StatusStamped::to_stdstring() const {
  std::stringstream ss;
  ss << "name: " << name_ << ", "
     << "created_at: " << timestamp_ << ", "
     << "x: " << pose_status_.pose.position.x << ", "
     << "y: " << pose_status_.pose.position.y << ", "
     << "z: " << pose_status_.pose.position.z << ", "
     << "qx: " << pose_status_.pose.orientation.x << ", "
     << "qy: " << pose_status_.pose.orientation.y << ", "
     << "qz: " << pose_status_.pose.orientation.z << ", "
     << "qw: " << pose_status_.pose.orientation.w << " \n";

  return ss.str();
}
