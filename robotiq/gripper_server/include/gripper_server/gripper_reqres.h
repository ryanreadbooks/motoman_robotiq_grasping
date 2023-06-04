#pragma once

const int ReqGripperManualAction      = 2000;       // 手动动作，指定宽度、速度、力
const int ReqGripperOpen              = 2001;       // 简单一个open动作
const int ReqGripperClose             = 2002;       // 简单一个close动作
const int ReqGripperStop              = 2003;       // 夹爪停止
const int ReqGripperERelease          = 2004;       // 紧急释放夹爪
const int ReqGetGripperState          = 2005;       // 获取夹爪当前的状态
const int ReqGripperDebug             = 2010;       // 调试功能