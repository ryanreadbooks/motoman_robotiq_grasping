#pragma once

#include <string>
#include <unordered_map>
#include <vector>

// 内置的预定义的token
// 定义motion cmd 脚本支持的命令
const std::string CMDSTART = "CMDSTART";
const std::string CMDEND = "CMDEND";

const std::string MOVETO = "MOVETO";
const std::string LIFTUP = "LIFTUP";
const std::string PAUSEFOR = "PAUSEFOR";
const std::string GRASPOPEN = "GRASPOPEN";
const std::string GRASPCLOSE = "GRASPCLOSE";

const int CMDSTART_CODE = 9000;
const int CMDEND_CODE = 9001;
const int MOVETO_CODE = 9002;
const int PAUSEFOR_CODE = 9003;
const int GRASPOPEN_CODE = 9004;
const int GRASPCLOSE_CODE = 9005;
const int LIFTUP_CODE = 9006;

// 命令语法：
// CMDSTART
// CMDEND
// MOVETO 目标点名字
// PAUSEFOR 需要暂停的毫秒数
// GRASPOPEN
// GRASPCLOSE
// LIFTUP

struct RealCmd {
  int opcode;
  std::vector<std::string> args;
};

class MotionCommandParser {
 public:
  MotionCommandParser();

  // 解析一行命令，解析成功返回true，解析失败返回false
  bool parse_line(const std::string& cmdline);

  std::vector<RealCmd>& get_parsed_motion_sequence() {
    return motion_sequence_;
  }

 private:
  std::unordered_map<std::string, int> cmd_ops_mapping_;
  std::unordered_map<std::string, int> expected_cmd_arg_size_;
  // 存储按照顺序解析出来的脚本命令
  std::vector<RealCmd> motion_sequence_;
};