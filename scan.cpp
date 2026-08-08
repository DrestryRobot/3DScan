// Scan.cpp
#include "Scan.h"
#include <QDebug>
#include <QFile>
#include <QTextStream>
#include <QElapsedTimer>

// ============================================
// 全局变量定义
// ============================================
double amp[64] = {0};
double tof[64] = {0};
double si = 0.0;
int beam = 49;
double robot_x = 0, robot_y = 0, robot_z = 0;
double robot_a = 0, robot_b = 0, robot_c = 0;
quint32 robot_ipoc = 0;
double longmen[2] = {0, 0};
bool m_start = false;

// ============================================
// Scan 实现
// ============================================
Scan::Scan(QObject *parent)
    : QObject(parent)
    , m_currentIndex(0)
    , m_totalFrames(0)
    , m_colBEAM(-1)
    , m_colLX(-1)
    , m_colLY(-1)
    , m_colX(-1)
    , m_colSI(-1)
    , m_colAMP_1(-1)
    , m_colTOF_1(-1)
    , m_isRunning(false)
{
    m_timer = new QTimer(this);
    m_timer->setTimerType(Qt::PreciseTimer);
    connect(m_timer, &QTimer::timeout, this, &Scan::sendNextFrame);
}

Scan::~Scan()
{
    m_timer->stop();
}

// ============================================
// CSV 解析
// ============================================
std::vector<std::string> Scan::parseCSVLine(const std::string& line)
{
    std::vector<std::string> result;
    std::string cell;
    bool inQuotes = false;

    for (char ch : line) {
        if (ch == '"') {
            inQuotes = !inQuotes;
        } else if (ch == ',' && !inQuotes) {
            result.push_back(cell);
            cell.clear();
        } else {
            cell += ch;
        }
    }
    result.push_back(cell);
    return result;
}

double Scan::safe_stod(const std::string& str, double defaultVal)
{
    std::string trimmed = str;
    trimmed.erase(0, trimmed.find_first_not_of(" \t\n\r"));
    trimmed.erase(trimmed.find_last_not_of(" \t\n\r") + 1);

    if (trimmed.empty()) {
        return defaultVal;
    }

    try {
        return std::stod(trimmed);
    } catch (const std::exception&) {
        return defaultVal;
    }
}

// ============================================
// 加载CSV文件
// ============================================
bool Scan::loadCSVFile(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Cannot open file:" << filename;
        return false;
    }

    m_data.clear();
    m_columnMap.clear();

    QTextStream stream(&file);

    // 读取表头
    QString headerLine = stream.readLine();
    if (headerLine.isEmpty()) {
        file.close();
        return false;
    }

    std::vector<std::string> headers = parseCSVLine(headerLine.toStdString());

    // 建立列索引映射
    for (size_t i = 0; i < headers.size(); i++) {
        m_columnMap[headers[i]] = static_cast<int>(i);
    }

    auto get = [this](const std::string& name) -> int {
        auto it = m_columnMap.find(name);
        return (it != m_columnMap.end()) ? it->second : -1;
    };

    m_colBEAM  = get("BEAM");
    m_colLX    = get("LX");
    m_colLY    = get("LY");
    m_colX     = get("X");
    m_colSI    = get("SI");
    m_colAMP_1 = get("AMP_1");
    m_colTOF_1 = get("TOF_1");

    // 读取数据
    while (!stream.atEnd()) {
        QString line = stream.readLine();
        if (!line.isEmpty()) {
            std::vector<std::string> cells = parseCSVLine(line.toStdString());
            if (cells.size() > 10) {
                m_data.push_back(cells);
            }
        }
    }
    file.close();

    m_totalFrames = m_data.size();
    m_currentIndex = 0;

    qDebug() << "Loaded" << m_totalFrames << "frames from CSV";
    return m_totalFrames > 0;
}

// ============================================
// 更新全局变量
// ============================================
void Scan::updateGlobalVariables(const std::vector<std::string>& cells)
{
    // 读取波束
    int beamVal = 49;
    if (m_colBEAM != -1 && m_colBEAM < (int)cells.size()) {
        beamVal = static_cast<int>(safe_stod(cells[m_colBEAM], 49));
    }
    beam = (beamVal > 0 && beamVal <= 64) ? beamVal : 49;

    // 读取龙门坐标
    double lx = 0.0, ly = 0.0;
    if (m_colLX != -1 && m_colLX < (int)cells.size()) {
        lx = safe_stod(cells[m_colLX], 0.0);
    }
    if (m_colLY != -1 && m_colLY < (int)cells.size()) {
        ly = safe_stod(cells[m_colLY], 0.0);
    }
    longmen[0] = lx;
    longmen[1] = ly;

    // 读取机器人位姿
    double pose[6] = {0, 0, 0, 0, 0, 0};
    if (m_colX != -1) {
        for (int i = 0; i < 6 && (m_colX + i) < (int)cells.size(); i++) {
            pose[i] = safe_stod(cells[m_colX + i], 0.0);
        }
    }
    robot_x = pose[0] + ly;
    robot_y = pose[1] - lx;
    robot_z = pose[2];
    robot_a = pose[3];
    robot_b = pose[4];
    robot_c = pose[5];

    // 读取SI
    if (m_colSI != -1 && m_colSI < (int)cells.size()) {
        si = safe_stod(cells[m_colSI], 0.0);
    } else {
        si = 0.0;
    }

    // 读取AMP和TOF
    if (m_colAMP_1 != -1 && m_colTOF_1 != -1) {
        for (int i = 0; i < beam; i++) {
            int ampIndex = m_colAMP_1 + 2 * i;
            int tofIndex = m_colTOF_1 + 2 * i;

            amp[i] = (ampIndex < (int)cells.size()) ? safe_stod(cells[ampIndex], 0.0) : 0.0;
            tof[i] = (tofIndex < (int)cells.size()) ? safe_stod(cells[tofIndex], 0.0) : 0.0;
        }
    }

    // 数据更新标志
    robot_ipoc++;
}

// ============================================
// 发送下一帧 (250Hz)
// ============================================
void Scan::sendNextFrame()
{
    if (m_currentIndex >= m_totalFrames) {
        m_timer->stop();
        m_isRunning = false;
        m_start = false;
        emit finished();
        qDebug() << "Playback finished";
        return;
    }

    // 更新全局变量
    updateGlobalVariables(m_data[m_currentIndex]);
    m_currentIndex++;

    // 发送帧数据
    ScanFrame frame;
    frame.ipoc = robot_ipoc;
    frame.si = si;
    frame.beam = beam;
    memcpy(frame.amp, amp, sizeof(double) * 64);
    memcpy(frame.tof, tof, sizeof(double) * 64);
    frame.pose[0] = robot_x;
    frame.pose[1] = robot_y;
    frame.pose[2] = robot_z;
    frame.pose[3] = robot_a;
    frame.pose[4] = robot_b;
    frame.pose[5] = robot_c;

    emit newFrameAvailable(frame);

    // // ✅ 打印实时数据
    // qDebug() << "Frame:" << frame.ipoc
    //          << "SI:" << frame.si
    //          << "Beam:" << frame.beam
    //          << "Pose:" << frame.pose[0] << frame.pose[1] << frame.pose[2]
    //          << frame.pose[3] << frame.pose[4] << frame.pose[5]
    //          << "AMP[0]:" << frame.amp[0]
    //          << "TOF[0]:" << frame.tof[0];

    // 在 Scan::sendNextFrame() 里
    static int dataCount = 0;
    static QElapsedTimer dataTimer;
    if (!dataTimer.isValid()) {
        dataTimer.start();
    }

    dataCount++;
    if (dataTimer.elapsed() >= 1000) { // 每10秒
        double dataFps = dataCount;
        qDebug() << "数据播放频率:" << dataFps << "fps";
        dataCount = 0;
        dataTimer.restart();
    }

}

// ============================================
// 开始数据采集
// ============================================
void Scan::start()
{
    if (m_isRunning || m_data.empty()) {
        return;
    }

    m_isRunning = true;
    m_currentIndex = 0;
    robot_ipoc = 0;
    m_start = true;

    m_timer->start(4);  // 250Hz = 4ms

    qDebug() << "Data acquisition started at 250Hz";
}

// ============================================
// 停止数据采集
// ============================================
void Scan::stop()
{
    if (m_isRunning) {
        m_timer->stop();
        m_isRunning = false;
        m_start = false;
        emit finished();
        qDebug() << "Scan stopped";
    }
}

// ============================================
// 判断是否已加载文件
// ============================================
bool Scan::isFileLoaded() const
{
    return !m_data.empty();
}
