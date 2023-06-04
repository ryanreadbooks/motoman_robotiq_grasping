#pragma once

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>

#include "coordinator/AutoGraspAction.h"
#include "coordinator/AutoGraspApi.h"
#include "coordinator/PredefinedMotionAction.h"
#include "coordinator/PredefinedMotionApi.h"
#include "coordinator/TagStatusApi.h"
#include "coordinator/motion_cmd_parser.h"
#include "coordinator/status_stamped.h"
#include "detection/DetectionResult.h"
#include "gripper_server/GripperService.h"
#include "gripper_server/gripper_reqres.h"
#include "ma2010_server/MA2010Service.h"
#include "ma2010_server/ma2010_reqres.h"

using namespace ros;
using namespace coordinator;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using geometry_msgs::Vector3;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

using gripper_server::GripperService;
using ma2010_server::MA2010Service;
using std_srvs::SetBool;
using std_srvs::Trigger;
using detection::DetectionResult;
using detection::DetectionResultConstPtr;
using actionlib::SimpleActionClient;
using actionlib::SimpleActionServer;

// 需要用到的服务和话题名
const static string MA2010_SERVICE_NAME = "/node_ma2010_service";
const static string GRIPPER_SERVICE_NAME = "/node_gripper_service";

// 需要订阅的话题名字
const static string DETECTION_SERVICE_NAME = "/detection/switch_service";
const static string DETECTION_TOPIC_RESULT_NAME = "/detection/result";
const static string DETECTION3D_TOPIC_RESULT_NAME = "/detection_3d/result";
const static string DETECTION_TOPIC_GRASPVIS_NAME = "/detection/grasp_vis";
const static string DETECTION3D_TOPIC_GRASPVIS_NAME = "/detection_3d/grasp_vis";
const static string DETECTION_TOPIC_PREGRASPVIS_NAME = "/detection/pre_grasp_vis";
const static string DETECTION3D_TOPIC_PREGRASPVIS_NAME = "/detection_3d/pre_grasp_vis";

// 对外提供的service服务名
const static string COORDINATOR_SWITCH_MODE_SERVICE_API = "/coordinator/switch_mode_api";
const static string COORDINATOR_AUTO_GRASP_SERVICE_API = "/coordinator/auto_grasp_api";
const static string COORDINATOR_PREDEFINED_MOTION_SERVICE_API = "/coordinator/predefined_motion_api";
const static string COORDINATOR_TAG_STATUS_SERVICE_API = "/coordinator/tag_status_api";
const static string COORDINATOR_RUN_ONCE_API = "/coordinator/run_once_api";
const static string COORDINATOR_SNAPSHOT_API = "/coordinator/snapshot_api";

// action
const static string COORDINATOR_AUTO_GRASP_ACTION_NAME = "/coordinator/auto_grasp_action";
const static string COORDINATOR_PREDEFINED_MOTION_ACTION_NAME = "/coordinator/predefined_motion_action";

const static string DEBUG_PARAM_NAME = "node_coordinator/debug_mode";

const static string DETECTION_ORIGIN_1 = "1#";
const static string DETECTION_ORIGIN_2 = "2#";

const static int SLEEP_USECS = 500 * 1000;  // 0.5s
const static double MIN_Z_ALLOWED = 0.365;  // 0.365m

// tag service supported opcode
const static int TAG_SERVICE_ADD = 8000;
const static int TAG_SERVICE_DEL = 8010;
const static int TAG_SERVICE_QUERY = 8020;

// builtin moveto target
const static string MOVETO_BUILTIN_ORIGIN = "DetectionOrigin";
const static string MOVETO_BUILTIN_DESTINATION = "Destination";

// 预定义的位置
static std::unordered_map<std::string, int> predefined_motion_movedest = {{MOVETO_BUILTIN_ORIGIN, 1},
                                                                          {MOVETO_BUILTIN_DESTINATION, 1}};

class Coordinator {
  using AutoGraspServer = SimpleActionServer<AutoGraspAction>;
  using AutoGraspClient = SimpleActionClient<AutoGraspAction>;

  using PredefinedMotionServer = SimpleActionServer<PredefinedMotionAction>;
  using PredefinedMotionClient = SimpleActionClient<PredefinedMotionAction>;

  enum class GripperOp { OPEN, CLOSE, GET_STATE, MANUAL };

 public:
  enum class DetectionMethod { Planar, Spatial };

  Coordinator(const DetectionMethod &dtype = DetectionMethod::Planar,
              const std::string &origin = DETECTION_ORIGIN_1);
  Coordinator(const Coordinator &) = delete;
  Coordinator &operator=(const Coordinator &) = delete;

 public:
  void detection_result_cb(const DetectionResultConstPtr &pd);
  void store_graspmarker_cb(const visualization_msgs::MarkerArrayConstPtr &md);
  void store_pregraspmarker_cb(const visualization_msgs::MarkerArrayConstPtr &md);

  // 只运行一次
  bool run_once(std::string &outmsg);

  // 切换模式服务函数
  bool do_switch_mode_service_api(SetBool::Request &req, SetBool::Response &res);

  // 调试模式下，运行一次
  bool do_run_once_service_api(Trigger::Request &req, Trigger::Response &res);

  // 调试模式下，snapshot保存一个结果
  bool do_snapshot_api(Trigger::Request &req, Trigger::Response &res);

  // 自动抓取的action的处理函数
  void do_auto_grasp_action_request(const AutoGraspServer::GoalConstPtr &goal);
  // 处理开始或者停止自动作业的函数
  bool do_auto_grasp_service_api(AutoGraspApi::Request &req, AutoGraspApi::Response &res);

  // 记录状态的服务函数
  bool do_tag_status_service_api(TagStatusApi::Request &req, TagStatusApi::Response &res);

  // 处理预动作action的处理函数
  void do_predefined_motion_action_request(const PredefinedMotionServer::GoalConstPtr &goal);
  // 处理开始停止自动motion的函数
  bool do_predefined_motion_service_api(PredefinedMotionApi::Request &req,
                                        PredefinedMotionApi::Response &res);

  // 将已经保存的status tag落盘
  void dump_status_records();

  void abort_predefined_motion(const std::string &msg);

  void abort_auto_grasp(const std::string &msg);

  // 检验MOVETO指令的target是否合法
  bool is_moveto_cmd_target_valid(const std::string &target);

  bool is_all_moveto_target_valid(const std::vector<RealCmd> &sequence);

 private:
  //  初始化Coordinator节点
  void init_coordinator_ros();

  void init_status_holder();

  void init_client_subpub();

  void init_api_service();

  void init_action();

  void init_mode();

  // 操作机械臂
  MA2010Service::Response operate_arm(int, Pose &, const string &data = "");
  // 回到检测原点位置
  bool back_to_origin();
  // 前往抓取目标位置
  bool go_to_target_position(const geometry_msgs::TransformStamped &, DetectionResultConstPtr target_det,
                             geometry_msgs::TransformStamped *pre_trans=nullptr);
  // 前往释放点
  bool go_to_destination();
  // 抬起机械臂
  bool lift_up_arm();

  // 操作夹爪
  GripperService::Response operate_gripper(GripperOp, double);
  // 打开夹爪
  bool open_gripper();
  // 关闭夹爪
  bool close_gripper();
  // 将夹爪宽度设置到设定宽度
  bool set_gripper_width(double);
  // 检查两指间是否有物体
  bool obj_detected_between_fingers();

  // 开启检测功能
  void enable_detection();
  // 关闭检测功能
  void disable_detection();

  // snapshot cache 辅助函数
  bool is_cache_valid() const;

  void clear_all_cache();

  void repaint_cache_graspvis();

  void reset_detection_result();

 private:
  // 标记是否为调试模式
  bool is_debug_ = true;
  // 在自动模式下，标记是否正在运行
  bool is_auto_running_ = false;
  std::string origin_location_;
  // 进行的检测类型 (planar or spatial)
  DetectionMethod detection_method_;
  NodeHandle handle_;
  ServiceClient ma2010_client_;
  ServiceClient gripper_client_;
  ServiceClient detection_client_;
  // 标记当前的检测功能是否可用
  bool detection_on_ = true;
  Subscriber detection_res_sub_;
  Subscriber detection_graspvis_sub_;
  Subscriber detection_pregraspvis_sub_;

  // 切换模式服务
  ServiceServer switch_mode_api_server_;
  // 调试模式下，触发运动一次的服务
  ServiceServer run_once_api_server_;
  // 调试模式下，snapshot检测结果的服务器
  ServiceServer snapshot_api_server_;

  // 开启或者停止自动作业的服务
  ServiceServer auto_grasp_api_server_;
  // 自动抓取的action服务
  std::shared_ptr<AutoGraspServer> auto_grasp_server_;
  AutoGraspResult ac_result_;
  AutoGraspFeedback ac_feedback_;
  // action客户端, 用来调用自动抓取action
  AutoGraspClient auto_grasp_client_;

  // 调用预定义action的对外服务
  ServiceServer predefined_motion_api_server_;
  // 执行预定义动作的action服务
  std::shared_ptr<PredefinedMotionServer> predefined_motion_server_;
  PredefinedMotionResult pd_result_;
  PredefinedMotionFeedback pd_feedback_;
  // action客户端，用来调用预动作action
  PredefinedMotionClient predefined_motion_client_;

  // 监听TF
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> p_tf_listener_;

  // 检测结果对象的缓存
  DetectionResultConstPtr p_detection_result_;
  // 检测结果对象的snapshot缓存
  DetectionResultConstPtr p_detection_result_cached_;           // 用于snapshot_api
  std::shared_ptr<TransformStamped> p_tf_grasppose_cached_;     // 用于snapshot_api,缓存真实抓取姿态
  std::shared_ptr<TransformStamped> p_tf_pregrasppose_cached_;  // 用于snapshot_api,缓存预抓取点姿态

  // 存储grasp_vis的可视化结果,方便调试
  visualization_msgs::MarkerArrayConstPtr p_graspmarker_;    // 会在回调中不断更新
  visualization_msgs::MarkerArrayPtr p_graspmarker_cached_;  // snapshot
  visualization_msgs::MarkerArrayConstPtr p_pregraspmarker_;
  visualization_msgs::MarkerArrayPtr p_pregraspmarker_cached_;

  Publisher snapshot_graspvis_pub_;
  Publisher snapshot_pregraspvis_pub_;

  // 状态存储
  StatusStampedHolder status_holder_;
  // 提供状态存储服务的服务器
  ServiceServer status_tagging_server_;

  // 存储文件夹的绝对路径
  std::string repository_path_;
};