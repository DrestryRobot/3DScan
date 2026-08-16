#ifndef DATAPANEL_H
#define DATAPANEL_H

#include <QFrame>
#include <QString>

class QLabel;

// 侧边浮动数据面板：统一显示 状态 / 最新帧 / 测量 三部分数据。
// 后续需要扩展字段时，直接在此类中增加标签与对应的 setter 即可。
class DataPanel : public QFrame
{
    Q_OBJECT

public:
    explicit DataPanel(QWidget *host = nullptr);

    // 将面板锚定到参考控件（如 VTK 绘制窗口）的右上角，浮动在其上方
    void anchorTo(QWidget *ref, int margin = 8);

    // 状态区：数据帧率 / 渲染帧率 / 扫描时间 / 点云点数
    void setStatus(int dataFps, int renderFps, const QString &scanTime, int pointCount);

    // 最新帧区：Frame / SI / Beam / Pose(6) / AMP[0] / TOF[0]
    void setFrameInfo(quint32 ipoc, double si, int beam,
                      const double pose[6], double amp0, double tof0);

    // 测量区：AMP / TOF / Distance
    void setMeasure(double amp, double tof, double distance);

    // 复位为初始占位文本
    void reset();

private:
    // 状态
    QLabel *m_lblDataFps = nullptr;
    QLabel *m_lblRenderFps = nullptr;
    QLabel *m_lblScanTime = nullptr;
    QLabel *m_lblPointCount = nullptr;

    // 最新帧
    QLabel *m_lblFrame = nullptr;
    QLabel *m_lblSi = nullptr;
    QLabel *m_lblBeam = nullptr;
    QLabel *m_lblPose = nullptr;
    QLabel *m_lblAngle = nullptr;
    QLabel *m_lblAmp0 = nullptr;
    QLabel *m_lblTof0 = nullptr;

    // 测量
    QLabel *m_lblAmp = nullptr;
    QLabel *m_lblTof = nullptr;
    QLabel *m_lblDist = nullptr;
};

#endif // DATAPANEL_H
