#pragma once

const int ReqGoHome             = 3000;  // 回到原点
const int ReqGoDest             = 3010;  // 前往预设定目标位置
const int ReqGoUp               = 3011;  // 机械臂仅在z轴方向提升预设距离
const int ReqGoDown             = 3012;  // 机械臂仅在z轴方向下降预设距离
const int ReqGoDetectionOrigin  = 3020;   // 前往检测位置
const int ReqGoCustom           = 3030;  // 前往用户指定目标位置
const int ReqGoCustomWithPre    = 3031;  // 在前往指定位置前，会先前往一个预先的姿态点
const int ReqGetCurPose         = 3040;  // 获取当前机械臂的末端位姿
const int ReqGetCurJoints       = 3050;  // 获取当前机械臂的所有关节角度

const int ResOK                 = 200;
const int ResFail               = 400;