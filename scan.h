#ifndef SCAN_H
#define SCAN_H

#include <QObject>
#include <QTimer>
#include <QString>
#include <vector>
#include <map>
#include <cstring>   // for memset

// 全局变量声明
extern double amp[64];
extern double tof[64];
extern double si;
extern int beam;
extern double robot_x, robot_y, robot_z;
extern double robot_a, robot_b, robot_c;
extern quint32 robot_ipoc;
extern double longmen[2];
extern bool m_start;

struct ScanFrame {
    quint32 ipoc;
    double amp[64];
    double tof[64];
    double si;
    int beam;
    double pose[6];

    ScanFrame() : ipoc(0), si(0), beam(49) {
        memset(amp, 0, sizeof(amp));
        memset(tof, 0, sizeof(tof));
        memset(pose, 0, sizeof(pose));
    }
};

class Scan : public QObject
{
    Q_OBJECT

public:
    explicit Scan(QObject *parent = nullptr);
    ~Scan();

    // 加载CSV文件
    bool loadCSVFile(const QString& filename);

    // 开始数据采集 (250Hz)
    void start();

    // 停止数据采集 ✅ 新增
    void stop();

    // 判断是否已加载文件 ✅ 新增
    bool isFileLoaded() const;

signals:
    void newFrameAvailable(const ScanFrame& frame);
    void finished();

private slots:
    void sendNextFrame();

private:
    std::vector<std::string> parseCSVLine(const std::string& line);
    double safe_stod(const std::string& str, double defaultVal = 0.0);
    void updateGlobalVariables(const std::vector<std::string>& cells);

private:
    std::vector<std::vector<std::string>> m_data;
    std::map<std::string, int> m_columnMap;
    int m_currentIndex;
    int m_totalFrames;

    // 列索引
    int m_colBEAM, m_colLX, m_colLY, m_colX, m_colSI, m_colAMP_1, m_colTOF_1;

    QTimer* m_timer;
    bool m_isRunning;
};

#endif // SCAN_H
