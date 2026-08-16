#include "datapanel.h"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>

DataPanel::DataPanel(QWidget *host)
    : QFrame(host)
{
    setObjectName("dataPanel");

    // 统一排版参数：数据行铺满整个面板，行高/行距统一，随上下分隔线整体缩放
    const int kRowSpacing = 4;    // 数据项行距
    const int kColSpacing = 15;   // 键值列间距

    QVBoxLayout *root = new QVBoxLayout(this);
    root->setContentsMargins(12, 8, 12, 8);
    root->setSpacing(kRowSpacing);

    // 新建标签辅助函数：sectionTitle=标题 / key=字段名 / 默认=数值
    auto makeLabel = [this](const QString &text, const char *role) {
        QLabel *label = new QLabel(text, this);
        if (role) {
            label->setProperty(role, true);
            if (qstrcmp(role, "sectionTitle") == 0)
                label->setAlignment(Qt::AlignCenter);   // 模块标题居中
        }
        return label;
    };
    auto makeSeparator = [this]() {
        QFrame *line = new QFrame(this);
        line->setProperty("separator", true);
        line->setFixedHeight(1);
        return line;
    };
    // 一行数据项：键靠左、值靠右，整行铺满宽度，行高由根布局统一分配
    auto makeRow = [&](const QString &key, QLabel **valueOut) {
        QWidget *row = new QWidget(this);
        row->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        QHBoxLayout *hl = new QHBoxLayout(row);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->setSpacing(kColSpacing);
        QLabel *keyLabel = makeLabel(key, "key");
        hl->addWidget(keyLabel);
        QLabel *value = makeLabel(QString(), "value");
        value->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        value->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        hl->addWidget(value, 1);
        *valueOut = value;
        return row;
    };

    // ===== 状态 =====
    root->addWidget(makeLabel("状  态", "sectionTitle"), 0);
    root->addWidget(makeRow("数据帧率", &m_lblDataFps), 1);
    root->addWidget(makeRow("渲染帧率", &m_lblRenderFps), 1);
    root->addWidget(makeRow("扫描时间", &m_lblScanTime), 1);
    root->addWidget(makeRow("点云点数", &m_lblPointCount), 1);

    root->addWidget(makeSeparator(), 0);

    // ===== 当前帧 =====
    root->addWidget(makeLabel("当前帧", "sectionTitle"), 0);
    root->addWidget(makeRow("Frame", &m_lblFrame), 1);
    root->addWidget(makeRow("SI", &m_lblSi), 1);
    root->addWidget(makeRow("Beam", &m_lblBeam), 1);
    root->addWidget(makeRow("Pose", &m_lblPose), 1);
    root->addWidget(makeRow("Angle", &m_lblAngle), 1);
    root->addWidget(makeRow("AMP[0]", &m_lblAmp0), 1);
    root->addWidget(makeRow("TOF[0]", &m_lblTof0), 1);

    root->addWidget(makeSeparator(), 0);

    // ===== 测量 =====
    root->addWidget(makeLabel("测  量", "sectionTitle"), 0);
    root->addWidget(makeRow("AMP", &m_lblAmp), 1);
    root->addWidget(makeRow("TOF", &m_lblTof), 1);
    root->addWidget(makeRow("Distance", &m_lblDist), 1);

    reset();

    setStyleSheet(
        "QFrame#dataPanel {"
        "  background-color: rgba(18, 24, 38, 220);"
        "  border: 1px solid #2a4a72;"
        "  border-radius: 4px;"
        "}"
        "QFrame#dataPanel QLabel {"
        "  color: rgb(230, 235, 240);"
        "  font-size: 14pt;"
        "  background: transparent;"
        "  padding: 0px;"
        "}"
        "QFrame#dataPanel QLabel[sectionTitle=\"true\"] {"
        "  color: #8ec5ff;"
        "  font-size: 13pt;"
        "  font-weight: bold;"
        "  padding-bottom: 1px;"
        "}"
        "QFrame#dataPanel QLabel[key=\"true\"] {"
        "  color: rgba(180, 195, 215, 170);"
        "}"
        "QFrame#dataPanel QLabel[value=\"true\"] {"
        "  font-family: \"Consolas\";"
        "}"
        "QFrame#dataPanel QFrame[separator=\"true\"] {"
        "  background-color: rgba(90, 138, 191, 90);"
        "  border: none;"
        "}");
}

void DataPanel::anchorTo(QWidget *ref, int margin)
{
    if (!ref)
        return;
    QWidget *host = parentWidget();
    if (!host)
        return;
    adjustSize();
    QPoint topRight = ref->mapTo(host, QPoint(ref->width(), 0));
    move(topRight.x() - width() - margin, topRight.y() + margin);
    raise();
    show();
}

void DataPanel::setStatus(int dataFps, int renderFps, const QString &scanTime, int pointCount)
{
    m_lblDataFps->setText(QString("%1 fps").arg(dataFps, 4));
    m_lblRenderFps->setText(QString("%1 fps").arg(renderFps, 4));
    m_lblScanTime->setText(scanTime);
    m_lblPointCount->setText(QString("%1").arg(pointCount, 8));
}

void DataPanel::setFrameInfo(quint32 ipoc, double si, int beam,
                             const double pose[6], double amp0, double tof0)
{
    m_lblFrame->setText(QString("%1").arg(ipoc, 8));
    m_lblSi->setText(QString("%1").arg(si, 8, 'f', 2));
    m_lblBeam->setText(QString("%1").arg(beam, 4));
    m_lblPose->setText(QString("%1 %2 %3")
                           .arg(pose[0], 5, 'f', 0)
                           .arg(pose[1], 5, 'f', 0)
                           .arg(pose[2], 5, 'f', 0));
    m_lblAngle->setText(QString("%1 %2 %3")
                           .arg(pose[3], 5, 'f', 0)
                           .arg(pose[4], 5, 'f', 0)
                           .arg(pose[5], 5, 'f', 0));
    m_lblAmp0->setText(QString("%1").arg(amp0, 8, 'f', 2));
    m_lblTof0->setText(QString("%1").arg(tof0, 8, 'f', 2));
}

void DataPanel::setMeasure(double amp, double tof, double distance)
{
    m_lblAmp->setText(QString("%1").arg(amp, 8, 'f', 3));
    m_lblTof->setText(QString("%1").arg(tof, 8, 'f', 3));
    m_lblDist->setText(QString("%1").arg(distance, 8, 'f', 3));
}

void DataPanel::reset()
{
    const double zero[6] = {0, 0, 0, 0, 0, 0};
    setStatus(0, 0, "00:00:00", 0);
    setFrameInfo(0, 0.0, 0, zero, 0.0, 0.0);
    m_lblAmp->setText("--");
    m_lblTof->setText("--");
    m_lblDist->setText("--");
}
