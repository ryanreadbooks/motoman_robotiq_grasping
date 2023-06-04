#include <iostream>
#include <sstream>
#include <string>

#include "coordinator/motion_cmd_parser.h"

MotionCommandParser::MotionCommandParser() {
  cmd_ops_mapping_ = {{CMDSTART, CMDSTART_CODE}, {CMDEND, CMDEND_CODE},       {MOVETO, MOVETO_CODE},
                      {PAUSEFOR, PAUSEFOR_CODE}, {GRASPOPEN, GRASPOPEN_CODE}, {GRASPCLOSE, GRASPCLOSE_CODE},
                      {LIFTUP, LIFTUP_CODE}};

  expected_cmd_arg_size_ = {{CMDSTART, 1},  {CMDEND, 1},     {MOVETO, 2}, {PAUSEFOR, 2},
                            {GRASPOPEN, 1}, {GRASPCLOSE, 1}, {LIFTUP, 1}};
}

bool MotionCommandParser::parse_line(const std::string& cmdline) {
  // simply separated by space
  std::stringstream ss(cmdline);
  std::vector<std::string> tokens;
  std::string token;
  while (getline(ss, token, ' ')) {
    if (!token.empty()) {
      tokens.emplace_back(token);
    }
  }
  if (tokens.empty()) {
    return false;
  }
  const std::string& cmd = tokens[0];

  // 对应的命令不存在
  if (cmd_ops_mapping_.count(cmd) == 0) {
    std::cout << cmd << " is not supported\n";
    return false;
  }

  RealCmd r;
  r.opcode = cmd_ops_mapping_[cmd];
  // 检查携带参数是否合理
  if (tokens.size() != expected_cmd_arg_size_[cmd]) {
    std::cout << cmd << " invalid syntax\n";
    return false;
  }
  r.args = std::vector<std::string>(tokens.begin() + 1, tokens.end());
  motion_sequence_.emplace_back(r);

  return true;
}