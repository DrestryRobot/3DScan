#ifndef SCANDATA_H
#define SCANDATA_H

// ============================================================
// SoundScan 与 3DScan 共享的数据接口（单一事实来源）
//
// 约定：
//   1. 所有跨模块共享的扫描/机器人全局量统一在这里声明，
//      在 scandata.cpp 中定义，其他文件只 include 本头文件。
//   2. 变量命名与 C:\3DScan 保持一致（m_start、amp[64] 等），
//      保证 3DScan 的 mainwindow7/scan/vtkvboactor/algorithm
//      移植过来后无需改名即可链接。
//   3. 移植 3DScan 时需删除以下重复定义：
//      - scan.cpp 第 14~22 行的 amp/tof/si/beam/robot_*/robot_ipoc/longmen/m_start
//      - mainwindow7.cpp 第 885~892 行附近的本地 amp/tof/longmen 定义
// ============================================================

#include <QtGlobal>

// ---- 超声采集数据（每帧最多 64 波束）----
extern double amp[64];
extern double tof[64];
extern bool beamValid[64];
extern double si;
extern int beam;

// ---- 机器人位姿（KUKA RSI，由 udpserver 更新）----
extern double robot_x, robot_y, robot_z;
extern double robot_a, robot_b, robot_c;
extern quint32 robot_ipoc;

// ---- 龙门位置 ----
extern double longmen[2];

// ---- 3D 绘制启停标志（由 start3DChecker/PLC 信号控制）----
extern bool m_start;

#endif // SCANDATA_H
