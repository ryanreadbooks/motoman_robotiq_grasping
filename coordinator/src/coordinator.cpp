#include <unistd.h>
#include <fstream>
#include <thread>

#include <ros/package.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>

#include "configor/json.hpp"  // for json
#include "coordinator/coordinator.h"

using namespace configor;  // for json

void check_need_throw_run_time_error(bool ret, const std::string &message) {
  ROS_INFO("ret = %s", ret ? "True" : "False");
  if (!ret) {
    throw std::runtime_error(message);
  }
}

Coordinator::Coordinator(const DetectionMethod &dtype, const std::string &origin)
    : detection_method_(dtype),
      auto_grasp_client_(COORDINATOR_AUTO_GRASP_ACTION_NAME, true),
      predefined_motion_client_(COORDINATOR_PREDEFINED_MOTION_ACTION_NAME, true),
      origin_location_(origin) {
  repository_path_ = ros::package::getPath("coordinator") + "/repository";

  init_coordinator_ros();
  init_status_holder();
};

void Coordinator::init_status_holder() {
  ROS_INFO("Loading existing predefined.status.bin in %s", repository_path_.c_str());
  // status_holder_.load_from_json("predefined.status.json");
  std::string filename = repository_path_ + "/predefined.status.bin";
  if (status_holder_.load_from(filename)) {
    ROS_INFO("Loaded existing predefined file");
  } else {
    ROS_WARN("Can not load and parse predefined file");
  }
}

void Coordinator::init_mode() {
  // 初始化模式
  handle_.param<bool>(DEBUG_PARAM_NAME, is_debug_, true);
}

void Coordinator::init_client_subpub() {
  // 初始化服务client和subscriber
  ma2010_client_ = handle_.serviceClient<MA2010Service>(MA2010_SERVICE_NAME);
  gripper_client_ = handle_.serviceClient<GripperService>(GRIPPER_SERVICE_NAME);
  detection_client_ = handle_.serviceClient<SetBool>(DETECTION_SERVICE_NAME);

  detection_res_sub_ =
      handle_.subscribe(detection_method_ == DetectionMethod::Planar ? DETECTION_TOPIC_RESULT_NAME
                                                                     : DETECTION3D_TOPIC_RESULT_NAME,
                        5, &Coordinator::detection_result_cb, this);

  // 订阅grasp_vis话题
  detection_graspvis_sub_ =
      handle_.subscribe(detection_method_ == DetectionMethod::Planar ? DETECTION_TOPIC_GRASPVIS_NAME
                                                                     : DETECTION3D_TOPIC_GRASPVIS_NAME,
                        5, &Coordinator::store_graspmarker_cb, this);

  detection_pregraspvis_sub_ =
      handle_.subscribe(detection_method_ == DetectionMethod::Planar ? DETECTION_TOPIC_PREGRASPVIS_NAME
                                                                     : DETECTION3D_TOPIC_PREGRASPVIS_NAME,
                        5, &Coordinator::store_pregraspmarker_cb, this);

  snapshot_graspvis_pub_ =
      handle_.advertise<visualization_msgs::MarkerArray>("/coordinator/graspvis_snapshot", 5, true);

  snapshot_pregraspvis_pub_ =
      handle_.advertise<visualization_msgs::MarkerArray>("/coordinator/pregraspvis_snapshot", 5, true);

  p_tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
}

void Coordinator::init_api_service() {
  // 模式切换服务
  switch_mode_api_server_ = handle_.advertiseService(COORDINATOR_SWITCH_MODE_SERVICE_API,
                                                     &Coordinator::do_switch_mode_service_api, this);

  // 调试状态下运行一次这个功能的服务
  run_once_api_server_ =
      handle_.advertiseService(COORDINATOR_RUN_ONCE_API, &Coordinator::do_run_once_service_api, this);

  // 获取当前抓取位姿的snapshot的服务
  snapshot_api_server_ =
      handle_.advertiseService(COORDINATOR_SNAPSHOT_API, &Coordinator::do_snapshot_api, this);

  // 开启和关闭自动运行的服务
  auto_grasp_api_server_ = handle_.advertiseService(COORDINATOR_AUTO_GRASP_SERVICE_API,
                                                    &Coordinator::do_auto_grasp_service_api, this);

  // 预动作服务
  predefined_motion_api_server_ = handle_.advertiseService(
      COORDINATOR_PREDEFINED_MOTION_SERVICE_API, &Coordinator::do_predefined_motion_service_api, this);

  // 状态存储服务
  status_tagging_server_ = handle_.advertiseService(COORDINATOR_TAG_STATUS_SERVICE_API,
                                                    &Coordinator::do_tag_status_service_api, this);
}

void Coordinator::init_action() {
  auto_grasp_server_.reset(
      new AutoGraspServer(handle_, COORDINATOR_AUTO_GRASP_ACTION_NAME,
                          boost::bind(&Coordinator::do_auto_grasp_action_request, this, _1), false));
  auto_grasp_server_->start();
  ROS_INFO("auto_grasp_server_ on");

  predefined_motion_server_.reset(new PredefinedMotionServer(
      handle_, COORDINATOR_PREDEFINED_MOTION_ACTION_NAME,
      boost::bind(&Coordinator::do_predefined_motion_action_request, this, _1), false));

  predefined_motion_server_->start();
  ROS_INFO("predefined_motion_server_ on");
}

void Coordinator::init_coordinator_ros() {
  init_mode();
  init_client_subpub();
  init_api_service();
  init_action();
  clear_all_cache();
}

// 会在子线程中回调
void Coordinator::detection_result_cb(const DetectionResultConstPtr &pd) {
  if (pd->success) {
    p_detection_result_ = pd;
  }
}

// 订阅grasp_vis
void Coordinator::store_graspmarker_cb(const visualization_msgs::MarkerArrayConstPtr &md) {
  if (md == nullptr) {
    ROS_WARN("Coordinator::store_graspmarker_cb occurs nullptr");
  }
  p_graspmarker_ = md;
  if (p_graspmarker_cached_ != nullptr) {
    snapshot_graspvis_pub_.publish(p_graspmarker_cached_);
  }
}

// 订阅pre_grasp_vis
void Coordinator::store_pregraspmarker_cb(const visualization_msgs::MarkerArrayConstPtr &md) {
  if (md == nullptr) {
    ROS_WARN("Coordinator::store_pregraspmarker_cb occurs nullptr");
  }
  p_pregraspmarker_ = md;
  if (p_pregraspmarker_cached_ != nullptr) {
    snapshot_pregraspvis_pub_.publish(p_pregraspmarker_cached_);
  }
}

// 开启检测功能
void Coordinator::enable_detection() {
  SetBool::Request req;
  SetBool::Response res;
  req.data = true;
  bool enable_detection_ok = detection_client_.call(req, res);
  if (enable_detection_ok && res.success) {
    ROS_INFO("Enable detection again");
    detection_on_ = true;
  } else {
    ROS_WARN("Can not enable detection again");
  }
}

// 关闭检测功能
void Coordinator::disable_detection() {
  // 暂时关闭检测功能
  bool detection_disabled = false;
  SetBool::Request req;
  SetBool::Response res;
  req.data = false;
  bool disable_detection_ok = detection_client_.call(req, res);
  if (disable_detection_ok && res.success) {
    detection_disabled = true;
    detection_on_ = false;
    ROS_INFO("Disable detection");
  } else {
    ROS_WARN("Can not disable detection, continues detecting");
  }
}

void Coordinator::reset_detection_result() {
  p_detection_result_.reset();
  p_detection_result_ = nullptr;
}

// 只进行一次步进
bool Coordinator::run_once(std::string &outmsg) {
  // 步进一次优先使用p_det_res_snapshot_所存的结果，如果p_det_res_snapshot_为空，则转而使用p_det_res_所存的结果
  // 从检测结果中得到抓取目标并且前往
  bool res = false;
  DetectionResultConstPtr target_det;
  if (p_detection_result_cached_ != nullptr) {
    target_det = p_detection_result_cached_;
    ROS_INFO("det_res_snapshot will be used");
  } else {
    ROS_INFO("det_res will be used");
    target_det = p_detection_result_;
  }
  if (target_det != nullptr && target_det->success) {
    try {
      // 确认回原点再开始检测
      back_to_origin();
      usleep(SLEEP_USECS);
      TransformStamped trans;
      TransformStamped pre_trans;

      if (p_tf_grasppose_cached_ != nullptr) {
        // 如果有snapshot就用snapshot
        trans = *p_tf_grasppose_cached_;
        pre_trans = *p_tf_pregrasppose_cached_;
      } else {
        // 如果没有snapshot就实时拿
        trans = tf_buffer_.lookupTransform("base_link", "grasp_candidate", ros::Time(0.0));
        pre_trans = tf_buffer_.lookupTransform("base_link", "pre_grasp_pose", ros::Time(0));
      }
      double qw = trans.transform.rotation.w;
      double qx = trans.transform.rotation.x;
      double qy = trans.transform.rotation.y;
      double qz = trans.transform.rotation.z;
      double tx = trans.transform.translation.x;
      double ty = trans.transform.translation.y;
      double tz = trans.transform.translation.z;
      ROS_INFO("Grasp pose:  (qw, qx, qy, qz, tx, ty, tz) is below");
      ROS_INFO("[%f, %f, %f, %f, %f, %f, %f]", qw, qx, qy, qz, tx, ty, tz);
      // 预闭合
      res = set_gripper_width(target_det->grasp_width);
      check_need_throw_run_time_error(res, "Can not set gripper width");
      // usleep(SLEEP_USECS);
      // 暂时关闭检测功能
      if (is_auto_running_) {
        disable_detection();
      }
      // 前往抓取目的地
      res = go_to_target_position(trans, target_det, &pre_trans);
      check_need_throw_run_time_error(res, "Can not go to target position");
      usleep(SLEEP_USECS);
      // 可以从参数服务器中读出一个参数，表示是否进行抓取动作
      bool perform_grasp = true;
      // 如果想要设置不进行抓取操作，则可以rosparam set /coordinator/debug/perform_grasp
      // false;默认情况下就是执行抓取操作
      perform_grasp = handle_.param("/coordinator/debug/perform_grasp", true);
      if (is_debug_ && !perform_grasp) {
        return true;
      }
      // 闭合夹爪
      res = close_gripper();
      check_need_throw_run_time_error(res, "Can not close finger");
      usleep(SLEEP_USECS);
      // 抬起
      res = lift_up_arm();
      check_need_throw_run_time_error(res, "Can not lift up arm");
      sleep(2);  // 等待夹爪得到正确的状态
      // 检查是否成功夹取物体
      res = obj_detected_between_fingers();
      check_need_throw_run_time_error(res, "No object between fingers, grasp failed!!");
      usleep(SLEEP_USECS);
      // 前往目标地点
      res = go_to_destination();
      check_need_throw_run_time_error(res, "Can not go to destination");
      usleep(SLEEP_USECS);
      // 松开夹爪
      res = open_gripper();
      usleep(SLEEP_USECS);
      // 回到原点
      back_to_origin();
      // 重新打开检测功能
      if (!detection_on_) {
        enable_detection();
      }
      outmsg = "success";
      reset_detection_result();  // 必须重置存储的检测结果
    } catch (std::exception &ex) {
      ROS_ERROR("%s", ex.what());
      open_gripper();  // may throw exception due to gripper server done and cause crash
      back_to_origin();
      outmsg = ex.what();
      // 如果运行过程中抛了异常，同样需要重新使能检测
      if (!detection_on_) {
        enable_detection();
      }
      clear_all_cache();
      reset_detection_result();
      return false;
    }
    // 重置相关变量
    clear_all_cache();
    return true;
  }
  ROS_WARN("No detection result message is cached: target null? %d, target.success = %d",
           target_det == nullptr, target_det->success);
  outmsg = "No detection result message is cached...";
  return false;
}

// 调试模式和自动模式切换
bool Coordinator::do_switch_mode_service_api(SetBool::Request &req, SetBool::Response &res) {
  is_debug_ = req.data;  // 是否为debug模式
  res.success = true;
  res.message = is_debug_ ? "Successfully set to debug mode" : "Successfully set to auto mode";
  handle_.setParam(DEBUG_PARAM_NAME, is_debug_);
  // 自动模式下清理缓存
  clear_all_cache();
  return true;
}

bool Coordinator::do_run_once_service_api(Trigger::Request &req, Trigger::Response &res) {
  // 自动模式下，不能够单独call run_once
  if (!is_debug_) {
    res.success = false;
    res.message = "Can not call run_once service in auto mode";
    return true;
  }
  // 开始run_once，返回是否执行成功
  res.success = run_once(res.message);
  return true;
}

bool Coordinator::is_cache_valid() const {
  // bool valid = !(p_graspmarker_cached_ == nullptr || p_detection_result_cached_ == nullptr ||
  //                p_tf_grasppose_cached_ == nullptr ||
  //                (detection_method_ == DetectionMethod::Spatial && p_tf_pregrasppose_cached_ == nullptr));

  bool graspmarker_cached_valid = p_graspmarker_cached_ != nullptr;
  bool detection_result_cached_valid = p_detection_result_cached_ != nullptr;
  bool tf_grasppose_cached_valid = p_tf_grasppose_cached_ != nullptr;

  bool spatial_case_valid =
      detection_method_ == DetectionMethod::Planar ? true : (p_tf_pregrasppose_cached_ != nullptr);
  bool pregraspmarker_cached_valid =
      detection_method_ == DetectionMethod::Planar ? true : (p_pregraspmarker_cached_ != nullptr);

  bool valid = graspmarker_cached_valid && detection_result_cached_valid && tf_grasppose_cached_valid &&
               spatial_case_valid && pregraspmarker_cached_valid;

  if (!valid) {
    std::stringstream ss;
    ss << "can not snapshot grasp pose:"
       << " p_graspmarker_cached_ nullptr? " << (p_graspmarker_cached_ == nullptr)
       << " p_detection_result_cached_ nullptr?" << (p_detection_result_cached_ == nullptr)
       << " p_tf_grasppose_cached_ nullptr? " << (p_tf_grasppose_cached_ == nullptr)
       << " spatial p_tf_pregrasppose_cached_ nullptr? " << (p_tf_pregrasppose_cached_ == nullptr)
       << " spatial p_pregraspmarker_cached_ nullptr?" << (p_pregraspmarker_cached_ == nullptr);
    ROS_WARN("%s", ss.str().c_str());
  }

  return valid;
}

void Coordinator::clear_all_cache() {
  p_detection_result_cached_ = nullptr;
  p_tf_grasppose_cached_ = nullptr;
  p_tf_pregrasppose_cached_ = nullptr;
  p_graspmarker_cached_ = nullptr;
  p_pregraspmarker_cached_ = nullptr;
}

void Coordinator::repaint_cache_graspvis() {
  if (p_pregraspmarker_cached_ != nullptr) {
    for (visualization_msgs::Marker &marker : p_pregraspmarker_cached_->markers) {
      marker.color.r = 0.0f;
      marker.color.g = 0.8f;
      marker.color.b = 1.0f;
      marker.color.a = 0.9f;
    }
  }

  if (p_graspmarker_cached_ != nullptr) {
    for (visualization_msgs::Marker &marker : p_graspmarker_cached_->markers) {
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.9f;
    }
  }
}

visualization_msgs::MarkerArrayPtr create_graspvis_cache(visualization_msgs::MarkerArrayConstPtr ptr) {
  visualization_msgs::MarkerArrayPtr arr = boost::make_shared<visualization_msgs::MarkerArray>();
  for (auto &marker : ptr->markers) {
    arr->markers.push_back(marker);
  }
  return arr;
}

// snapshot一个检测结果并且保存
bool Coordinator::do_snapshot_api(Trigger::Request &req, Trigger::Response &res) {
  if (!is_debug_) {
    res.success = false;
    res.message = "Can not call snapshot service in auto mode";
    return true;
  }
  try {
    // 保存当前最近的那个grasp pose
    TransformStamped trans = tf_buffer_.lookupTransform("base_link", "grasp_candidate", ros::Time(0.0));
    p_tf_grasppose_cached_ = std::make_shared<TransformStamped>(trans);
    if (detection_method_ == DetectionMethod::Spatial) {
      // 如果是spatial的抓取策略的话，需要进行处理,额外缓存下pre_grasp_pose
      TransformStamped pre_grasp_pose =
          tf_buffer_.lookupTransform("base_link", "pre_grasp_pose", ros::Time(0.0));  // may throw
      p_tf_pregrasppose_cached_ = std::make_shared<TransformStamped>(pre_grasp_pose);
      p_pregraspmarker_cached_ = create_graspvis_cache(p_pregraspmarker_);
    }
  } catch (std::exception &what) {
    ROS_WARN("do_snapshot_api lookupTransform exception: %s", what.what());
    clear_all_cache();
  }

  p_detection_result_cached_ = p_detection_result_;
  // 保存当前的rviz中的抓取位姿示意，并且以一个新的话题发布出去（在后台线程中发送了，见detection_grasp_vis_cb方法）
  p_graspmarker_cached_ = create_graspvis_cache(p_graspmarker_);
  if (!is_cache_valid()) {
    // 只要有一个没有保存成功,就失败
    ROS_WARN("snapshot cache invalid");
    res.success = false;
    res.message = "can not save snapshot";
    ROS_WARN("Successfully snapshot current grasp pose status");
  } else {
    res.success = true;
    std::stringstream ss;
    ss << "method: " << p_detection_result_cached_->method << ", "
       << "success: " << p_detection_result_cached_->success << ", ";
    res.message = "snapshot is saved. " + ss.str();
    repaint_cache_graspvis();
    ROS_INFO("Successfully snapshot current grasp pose status");
  }
  return true;
}

void Coordinator::abort_auto_grasp(const std::string &msg) {
  ac_result_.success = false;
  ac_result_.message = msg;
  auto_grasp_server_->setAborted(ac_result_, ac_result_.message);
  is_auto_running_ = false;
  ROS_ERROR("Auto grasp aborted due to %s", msg.c_str());
}

// 在自动运行情况下，进行开启自动抓取物体的作业
void Coordinator::do_auto_grasp_action_request(const AutoGraspServer::GoalConstPtr &goal) {
  // 不是在自动模式下，不能自己动作
  if (is_debug_) {
    abort_auto_grasp(
        "Can not start auto running in debug mode, call service "
        "coordinator/switch_service and set data to false");
    return;
  }

  if (is_auto_running_) {
    abort_auto_grasp("auto running is on now, can not run another auto-work again");
    return;
  }

  ros::Time st = ros::Time::now();
  is_auto_running_ = true;
  ROS_INFO(
      "Triggered auto pick-and-place, number of objects to grasped is %d, "
      "max attempts is %d",
      goal->n_object, goal->max_attempts);

  unsigned int n_cur_grasped_objs = 0;  // 成功抓取的次数
  unsigned int cur_attempt = 0;         // 当前尝试的次数

  // 当还没有抓取完所有物体，并且还剩余抓取次数才能保持自动运行状态
  while (n_cur_grasped_objs < goal->n_object && cur_attempt < goal->max_attempts) {
    cur_attempt++;
    ROS_INFO("In auto running mode, now in attempt-%d", cur_attempt);
    std::string msg;
    bool res = run_once(msg);
    if (!res) {
      // 失败
      ROS_WARN("Attempt-%d failed, completion is (%d/%d)", cur_attempt, n_cur_grasped_objs, goal->n_object);
    } else {
      // 成功
      n_cur_grasped_objs++;
    }
    sleep(3);
    ROS_INFO("Attempt-%d finished, current completion is (%d/%d)", cur_attempt, n_cur_grasped_objs,
             goal->n_object);
    // 组织feedback返回结果
    ac_feedback_.n_cur_grasped = n_cur_grasped_objs;
    ac_feedback_.n_attempts_left = goal->max_attempts - cur_attempt;
    auto_grasp_server_->publishFeedback(ac_feedback_);
    if (auto_grasp_server_->isPreemptRequested()) {
      // 停止作业
      ROS_INFO("auto_grasp_server_ PreemptRequested, auto working will stop");
      ac_result_.success = true;
      ac_result_.n_success = n_cur_grasped_objs;
      ac_result_.message = "Operation stopped";
      auto_grasp_server_->setPreempted(ac_result_, ac_result_.message);
      is_auto_running_ = false;
      return;
    }
  }
  if (cur_attempt >= goal->max_attempts) {
    ROS_INFO("Max attempt reached!!!");
  }
  ac_result_.success = true;
  std::stringstream ss;
  double duration = (ros::Time::now() - st).toSec();
  ss << "Successfully grasped " << n_cur_grasped_objs << " objects out of " << goal->n_object
     << " objects in " << duration << " seconds";
  ac_result_.n_success = n_cur_grasped_objs;
  ac_result_.message = ss.str();

  ROS_INFO("Action took %.6f seconds", duration);
  is_auto_running_ = false;
  auto_grasp_server_->setSucceeded(ac_result_, ss.str());
}

// 读出ifstream中的所有内容并返回指针
std::string read_all(std::ifstream &ifs) {
  ifs.seekg(0, std::ios::end);
  size_t filesize = ifs.tellg();
  ifs.seekg(0, std::ios::beg);
  char *buf = new char[filesize];
  ifs.read(buf, filesize);
  string out(buf, filesize);
  delete buf;

  return out;
}

void Coordinator::abort_predefined_motion(const std::string &msg) {
  pd_result_.success = false;
  pd_result_.message = msg;
  predefined_motion_server_->setAborted(pd_result_, pd_result_.message);
  is_auto_running_ = false;
  ROS_ERROR("Abort motion due to %s", msg.c_str());
}

bool Coordinator::is_moveto_cmd_target_valid(const std::string &target) {
  if (target != MOVETO_BUILTIN_DESTINATION && target != MOVETO_BUILTIN_ORIGIN) {
    if (!status_holder_.has(target)) {
      return false;
    }
  }
  return true;
}

bool Coordinator::is_all_moveto_target_valid(const std::vector<RealCmd> &sequence) {
  bool ok = true;
  for (size_t i = 1; i < sequence.size() - 1; i++) {
    if (sequence[i].opcode == MOVETO_CODE && !is_moveto_cmd_target_valid(sequence[i].args[0])) {
      ok = false;
      ROS_WARN("MOVETO command destination %s is not supported at line %lu", sequence[i].args[0].c_str(), i);
    }
  }
  return ok;
}

// 自动模式下，可以按照执行预动作
void Coordinator::do_predefined_motion_action_request(const PredefinedMotionServer::GoalConstPtr &goal) {
  // 不是在自动模式下，不能自己动作
  if (is_debug_) {
    abort_predefined_motion(
        "Can not start auto running in debug mode, call service "
        "coordinator/switch_service and set data to false");
    return;
  }

  // 当前正在进行自动作业，同样不能再次启动自动作业
  if (is_auto_running_) {
    abort_predefined_motion("auto running is on now, can not run another auto-work again");
    return;
  }

  ros::Time st = ros::Time::now();
  is_auto_running_ = true;

  // 预动作文件解释器
  MotionCommandParser parser;
  // 文件名仅支持相对路径，在repository文件夹下
  std::string motion_file_path = repository_path_ + "/" + goal->filename;
  std::ifstream ifs(motion_file_path);
  if (!ifs) {
    abort_predefined_motion("can not open file: " + goal->filename);
    return;
  }

  // 读入文件的所有内容
  // 这里先不考虑性能问题
  string content = read_all(ifs);
  if (content.empty()) {
    abort_predefined_motion("file content is empty");
    return;
  }

  std::vector<std::string> cmdlines;
  std::stringstream ss(content);
  std::string cmdline;
  ROS_INFO("cmdlines are following:");
  while (std::getline(ss, cmdline)) {
    if (!cmdline.empty()) {
      cmdlines.emplace_back(cmdline);
      ROS_INFO("%s", cmdline.c_str());
    }
  }

  // 开始解析每一个指令
  for (size_t i = 0; i < cmdlines.size(); i++) {
    if (!parser.parse_line(cmdlines[i])) {
      abort_predefined_motion("can not parse command: " + cmdlines[i] + " at line " + std::to_string(i + 1));
      return;
    }
    if (i == 0 && parser.get_parsed_motion_sequence()[i].opcode != CMDSTART_CODE) {
      abort_predefined_motion("cmd is not beginning with CMDSTART");
      return;
    }
    if (i == (cmdlines.size() - 1) && parser.get_parsed_motion_sequence()[i].opcode != CMDEND_CODE) {
      abort_predefined_motion("cmd is not ending with CMDEND");
      return;
    }
  }

  // 获取指令并且执行
  auto &sequence = parser.get_parsed_motion_sequence();
  ROS_INFO("Totally we parsed %lu commands in %s", sequence.size(), goal->filename.c_str());
  if (!is_all_moveto_target_valid(sequence)) {
    abort_predefined_motion("not all the MOVETO targets are supported");
    return;
  }

  for (size_t i = 1; i < sequence.size() - 1; i++) {
    auto &s = sequence[i];
    ROS_INFO("Executing %lu-th command (%d)", i, s.opcode);
    // simple condition judgement
    if (s.opcode == MOVETO_CODE) {
      const std::string &destination = s.args[0];
      if (status_holder_.has(destination)) {
        //  status tag中找到了
        StatusStamped ss = status_holder_.datas[destination];
        Pose target_pose = ss.pose();
        bool ok = operate_arm(ReqGoCustom, target_pose).rescode;
        ROS_INFO("ReqGoCustom to %s", ss.get_name().c_str());
        if (!ok) {
          abort_predefined_motion("can not go to status-tag position: " + destination);
          return;
        }
      } else {
        // status tag中找不到
        // 在预定义的位置中找
        if (predefined_motion_movedest.count(destination) == 0) {
          abort_predefined_motion("the target is not defined");
          return;
        }
        bool ok = false;
        if (destination == MOVETO_BUILTIN_ORIGIN) {
          ok = back_to_origin();
          ROS_INFO("back to origin");
        } else if (destination == MOVETO_BUILTIN_DESTINATION) {
          ROS_INFO("go to destination");
          ok = go_to_destination();
        }
        if (!ok) {
          abort_predefined_motion("can not move to target");
          return;
        }
      }
    } else if (s.opcode == PAUSEFOR_CODE) {
      std::string mill_str = s.args[0];
      try {
        uint64_t millsec = std::stol(mill_str);
        ROS_INFO("sleeping for %lu millseconds", millsec);
        sleep(millsec / 1000);
      } catch (const std::exception &ex) {
        auto what = "duration for pause not known " + std::string(ex.what());
        abort_predefined_motion(what);
        return;
      }
    } else if (s.opcode == GRASPOPEN_CODE) {
      bool ok = open_gripper();
      ROS_INFO("open gripper");
      if (!ok) {
        abort_predefined_motion("can not open gripper");
        return;
      }
    } else if (s.opcode == GRASPCLOSE_CODE) {
      bool ok = close_gripper();
      ROS_INFO("close gripper");
      if (!ok) {
        abort_predefined_motion("can not close gripper");
        return;
      }
    } else if (s.opcode == LIFTUP_CODE) {
      bool ok = lift_up_arm();
      ROS_INFO("liftup arm");
      if (!ok) {
        abort_predefined_motion("can not lift up arm");
        return;
      }
    } else {
      abort_predefined_motion("cmd not supported");
      return;
    }
  organize_feedback:
    // 组织feedback返回结果
    pd_feedback_.index = i;
    pd_feedback_.message = "normal";
    predefined_motion_server_->publishFeedback(pd_feedback_);
    // 中途请求停止了
    if (predefined_motion_server_->isPreemptRequested()) {
      ROS_INFO("predefined_motion_server_ PreemptRequested, auto working will stop");
      pd_result_.success = true;
      pd_result_.message = "stop required";
      predefined_motion_server_->setPreempted(pd_result_, pd_result_.message);
      is_auto_running_ = false;
      return;
    }
    ROS_INFO("Finish motion-%lu", i);
  }
  // 所有的指令都跑完了，结束流程
  pd_result_.success = true;
  double duration = (ros::Time::now() - st).toSec();
  std::stringstream outss;
  outss << "All predefined motions have been done. Totally took time " << duration << " seconds";
  pd_result_.message = outss.str();
  is_auto_running_ = false;
  predefined_motion_server_->setSucceeded(pd_result_, outss.str());
  ROS_INFO("Finished predefined action in %.4f seconds", duration);
}

bool Coordinator::do_auto_grasp_service_api(AutoGraspApi::Request &req, AutoGraspApi::Response &res) {
  if (is_debug_) {
    string msg =
        "Can not start auto running in debug mode, call service "
        "/coordinator/switch_service and set data to false";
    res.success = false;
    res.message = msg;
    return true;
  }
  if (is_auto_running_ && req.data == "on") {
    res.success = false;
    res.message = "Already auto running";
  } else {
    if (req.data == "on") {
      // 开启
      AutoGraspGoal goal;
      goal.n_object = req.n_object;
      goal.max_attempts = req.max_attempts;
      goal.data = req.data;
      // 给action服务器发送目标
      auto_grasp_client_.sendGoal(goal);  // 非阻塞，直接返回
      res.message = "Auto grasp operation request sent";
    } else if (req.data == "off") {
      // 终止当前作业
      auto_grasp_client_.cancelGoal();
      res.message = "Stop operation requested, wait until current operation finishes";
    }
    res.success = true;
  }
  return true;
}

bool Coordinator::do_predefined_motion_service_api(PredefinedMotionApi::Request &req,
                                                   PredefinedMotionApi::Response &res) {
  // 这个服务函数里面调用action
  if (is_debug_) {
    string msg =
        "Can not start auto running in debug mode, call service "
        "/coordinator/switch_service and set data to false";
    res.success = false;
    res.message = msg;
    return true;
  }
  if (is_auto_running_ && req.data == "on") {
    res.success = false;
    res.message = "Already auto running";
  } else {
    if (req.data == "on") {
      PredefinedMotionGoal goal;
      goal.filename = req.filename;
      predefined_motion_client_.sendGoal(goal);  // 非阻塞，直接返回
      res.message = "Predefined motion request sent";
    } else {
      // 终止当前action
      predefined_motion_client_.cancelGoal();
      res.message = "Stop predefined operation, requested, wait until current operation finishes";
    }
    res.success = true;
  }

  return true;
}

void tag_service_fail(TagStatusApi::Response &res, const std::string &msg) {
  res.success = false;
  res.message = msg;
}

bool Coordinator::do_tag_status_service_api(TagStatusApi::Request &req, TagStatusApi::Response &res) {
  if (req.name.empty()) {
    tag_service_fail(res, "Tag current status failed, name is not allowed to be empty");
    return true;
  }
  if (req.name == MOVETO_BUILTIN_DESTINATION || req.name == MOVETO_BUILTIN_ORIGIN) {
    tag_service_fail(res, "Status name '" + req.name + "' is reserved, please specify another name");
    return true;
  }
  // 检验操作码
  int opcode = req.opcode;
  if (opcode == TAG_SERVICE_QUERY) {
    // 查询
    if (!status_holder_.has(req.name)) {
      tag_service_fail(res, "Can not find status " + req.name);
      return true;
    }
    // 存在
    auto &stat = status_holder_.datas[req.name];
    res.success = true;
    res.message = stat.to_stdstring();
  } else if (opcode == TAG_SERVICE_DEL) {
    // 删除
    // 查询
    if (!status_holder_.has(req.name)) {
      tag_service_fail(res, "Can not find status " + req.name);
      return true;
    }
    // 存在
    status_holder_.datas.erase(req.name);
    res.success = true;
    res.message = "Del " + req.name + " ok";
  } else if (opcode == TAG_SERVICE_ADD) {
    // 添加
    if (status_holder_.has(req.name)) {
      tag_service_fail(res, "Tag current status failed, status with name " + req.name + " already exists");
      return true;
    } else {
      // 调用ma2010服务实时获取机械臂状态
      geometry_msgs::Pose ret_pose;
      // 获取成功
      auto ma2010res = operate_arm(ReqGetCurPose, ret_pose);
      if (ma2010res.rescode != ResOK) {
        // 获取失败
        res.success = false;
        res.message = "Tag current status failed: " + ma2010res.data;
      } else {
        // 获取成功
        ret_pose.position.x = ma2010res.curstate.pose.position.x;
        ret_pose.position.y = ma2010res.curstate.pose.position.y;
        ret_pose.position.z = ma2010res.curstate.pose.position.z;
        ret_pose.orientation.x = ma2010res.curstate.pose.orientation.x;
        ret_pose.orientation.y = ma2010res.curstate.pose.orientation.y;
        ret_pose.orientation.z = ma2010res.curstate.pose.orientation.z;
        ret_pose.orientation.w = ma2010res.curstate.pose.orientation.w;

        StatusStamped cur_status(req.name, ret_pose);
        status_holder_.add(req.name, cur_status);
        res.success = true;
        res.message = "Tag current status as \"" + req.name + "\"";
      }
    }
  } else {
    tag_service_fail(res, "Tag service opcode " + std::to_string(opcode) + " not supported");
  }

  return true;
}

void Coordinator::dump_status_records() {
  std::string filename = repository_path_ + "/predefined.status.bin";
  std::string filename_json = repository_path_ + "/predefined.status.json";
  // .bin文件保存的文件的浮点数精度可以得到保证
  status_holder_.save_to(filename);
  // .json保存的数据中的浮点数有精度损失，不适合做存储
  // 这里保存下来只是为了直观的看到保存了什么数据，因为.bin文件保存的数据看起来不直观
  status_holder_.save_to_json(filename_json);
}

// 回到检测原点位置
bool Coordinator::back_to_origin() {
  ROS_INFO("Go back to origin");
  Pose tp;
  int detection_origin =
      origin_location_ == DETECTION_ORIGIN_1 ? ReqGoDetectionOrigin : ReqGoDetectionOrigin2;
  bool ret = operate_arm(detection_origin, tp).rescode == ResOK;
  if (ret) {
    ROS_INFO("Reached origin");
    return true;
  }
  ROS_WARN("Can not reach origin");
  return false;
}

// 前往抓取目标位置
bool Coordinator::go_to_target_position(const geometry_msgs::TransformStamped &tp,
                                        DetectionResultConstPtr target_det,
                                        geometry_msgs::TransformStamped *pre_trans) {
  ROS_INFO("Going to grasp target position");
  Pose target_pose;
  target_pose.position.x = tp.transform.translation.x;
  target_pose.position.y = tp.transform.translation.y;
  target_pose.position.z = tp.transform.translation.z;
  target_pose.orientation.w = tp.transform.rotation.w;
  target_pose.orientation.x = tp.transform.rotation.x;
  target_pose.orientation.y = tp.transform.rotation.y;
  target_pose.orientation.z = tp.transform.rotation.z;
  string operate_data = "";
  if (detection_method_ == DetectionMethod::Spatial && target_det &&
      target_det->method == DetectionResult::METHOD_SPATIAL) {  // spatial detection有额外的参数需要传入
    json data_js;
    uint method = DetectionResult::METHOD_SPATIAL;
    // 如果是spatial的抓取策略的话，需要进行处理
    TransformStamped pre_grasp_pose;
    if (p_tf_pregrasppose_cached_ == nullptr) {
      // 从tf中直接获取预抓取点相对与机械臂基座的位姿
      if (pre_trans == nullptr) {
        pre_grasp_pose =
            tf_buffer_.lookupTransform("base_link", "pre_grasp_pose", ros::Time(0.0));  // TODO may throw
      } else {
        pre_grasp_pose = *pre_trans;
      }
    } else {
      // 有缓存的时候就用缓存
      pre_grasp_pose = *p_tf_pregrasppose_cached_;
    }
    data_js["method"] = method;
    data_js["pre_grasp_pose"] = {pre_grasp_pose.transform.translation.x,
                                 pre_grasp_pose.transform.translation.y,
                                 std::max(MIN_Z_ALLOWED, pre_grasp_pose.transform.translation.z)};
    operate_data = data_js.dump();  // 序列化为string
    ROS_INFO("in go_to_target_position, operate_data = %s", operate_data.c_str());
  } else {
    target_pose.position.z -= 0.025;
  }
  // 对target pose的高度进行一些限制
  target_pose.position.z = std::max(MIN_Z_ALLOWED, target_pose.position.z);
  // 统一进行机械臂的操作
  bool ret = operate_arm(ReqGoCustomWithPre, target_pose, operate_data).rescode == ResOK;
  if (ret) {
    ROS_INFO("Successfully moved to target pose");
    return true;
  }
  ROS_WARN("Can not move to target pose");
  return false;
}

// 前往释放点
bool Coordinator::go_to_destination() {
  ROS_INFO("Going to destination");
  Pose tp;
  bool ret = operate_arm(ReqGoDest, tp).rescode == ResOK;
  if (ret) {
    ROS_INFO("Reached destination");
    return true;
  }
  ROS_WARN("Can not reach destination");
  return false;
}

// 抬起机械臂
bool Coordinator::lift_up_arm() {
  // OPTIM　这里可以选择回到预抓取点，这样就可以适应两种情况
  Pose tp;
  bool ret = operate_arm(ReqGoUp, tp).rescode == ResOK;
  if (ret) {
    ROS_INFO("Successfully moved up");
    return true;
  }
  ROS_WARN("Can not move up");
  return false;
}

// 机械臂操作统一请求
MA2010Service::Response Coordinator::operate_arm(int op, Pose &target, const string &data) {
  MA2010Service ma_servant;
  ma_servant.request.reqcode = op;
  ma_servant.request.target = target;
  ma_servant.request.data = data;
  bool ret = ma2010_client_.call(ma_servant);
  return ma_servant.response;
}

GripperService::Response Coordinator::operate_gripper(GripperOp op, double width = 0.0) {
  GripperService grip_servant;
  if (op == GripperOp::OPEN) {
    grip_servant.request.reqcode = ReqGripperOpen;
  } else if (op == GripperOp::CLOSE) {
    grip_servant.request.reqcode = ReqGripperClose;
    grip_servant.request.speed = 0.7;
    grip_servant.request.force = 1.3;
  } else if (op == GripperOp::MANUAL) {
    grip_servant.request.position = width;
    grip_servant.request.speed = 1.0;
    grip_servant.request.force = 1.0;
    grip_servant.request.reqcode = ReqGripperManualAction;
  } else {
    grip_servant.request.reqcode = ReqGetGripperState;
  }
  bool ret = gripper_client_.call(grip_servant);

  return grip_servant.response;
}

// 打开夹爪
bool Coordinator::open_gripper() {
  int rescode = operate_gripper(GripperOp::OPEN).rescode;
  ROS_INFO("Open gripper rescode = %d", rescode);
  return rescode == ResOK;
}

bool Coordinator::close_gripper() {
  int rescode = operate_gripper(GripperOp::CLOSE).rescode;
  ROS_INFO("Close gripper rescode = %d", rescode);
  return rescode == ResOK;
}

bool Coordinator::set_gripper_width(double width) {
  int rescode = operate_gripper(GripperOp::MANUAL, width).rescode;
  ROS_INFO("Manual gripper rescode = %d", rescode);
  return rescode == ResOK;
}

bool Coordinator::obj_detected_between_fingers() {
  string res_data = operate_gripper(GripperOp::GET_STATE).data;
  if (res_data.size() == 0) {
    return false;
  }
  json j = json::parse(res_data);
  return (bool)j["obj_detected"];
}