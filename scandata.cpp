#include "scandata.h"

// 超声采集数据
double amp[64] = {0};
double tof[64] = {0};
double si = 0.0;
int beam = 49;                     // 与 3DScan 默认波束数一致

// 机器人位姿
double robot_x = 0, robot_y = 0, robot_z = 0;
double robot_a = 0, robot_b = 0, robot_c = 0;
quint32 robot_ipoc = 0;

// 龙门位置
double longmen[2] = {0, 0};

// 3D 绘制启停标志
bool m_start = false;
