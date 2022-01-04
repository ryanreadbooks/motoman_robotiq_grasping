#pragma once

const int ReqGoHome = 300;  // 回到原点
const int ReqGoDest = 301;  // 前往预设定目标位置
const int ReqGoDetectionOrigin = 302;// 前往检测位置
const int ReqGoCustom = 303;  // 前往用户指定目标位置
const int ReqGetCurPose = 304;// 获取当前机械臂的末端位姿
const int ReqGetCurJoints = 305;// 获取当前机械臂的所有关节角度

const int ResOK = 200;
const int ResFail = 400;