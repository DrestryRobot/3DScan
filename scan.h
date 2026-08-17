#ifndef SCAN_H
#define SCAN_H

#include <QObject>
#include <QTimer>
#include <QString>
#include <QMetaType>
#include <QElapsedTimer>
#include <QFile>
#include <QByteArray>
#include <vector>
#include <array>
#include <map>
#include <fstream>
#include <unordered_map>
#include <deque>
#include <cstring>   // for memset
#include <atomic>

// 全局变量声明
extern double amp[64];
extern double tof[64];
extern bool beamValid[64];
extern double si;
extern int beam;
extern double robot_x, robot_y, robot_z;
extern double robot_a, robot_b, robot_c;
extern quint32 robot_ipoc;
extern double longmen[2];
extern bool m_start;

// x
// 均匀网格重采样后的单点（世界坐标 + 颜色数据）
struct GridPoint {
    float x, y, z;
};

// 均匀网格重采样后的一行（对应播放中的一帧）
struct GridFrame {
    double pose[6];      // 中心线插值位姿，仅用于界面显示
    float si;
    int valid;           // 有效点数（通常49）
    int passIndex;       // 所属带序号（曲面网格按带分行）
    std::array<GridPoint, 49> pts;
    std::array<float, 49> amp;
    std::array<float, 49> tof;
    std::array<int, 49> gk;
    std::array<int, 49> gj;

    GridFrame() : si(0), valid(0), passIndex(0) {
        memset(pose, 0, sizeof(pose));
        for (int i = 0; i < 49; i++) {
            pts[i].x = pts[i].y = pts[i].z = 0;
            amp[i] = tof[i] = 0;
            gk[i] = gj[i] = 0;
        }
    }
};

struct ScanFrame {
    quint32 ipoc;
    double amp[64];
    double tof[64];
    double beamZ[64];
    double si;
    int beam;
    double pose[6];
    double lx = 0, ly = 0;
    const float* worldXYZ;
    int worldCount;
    bool hasWorld;
    const int* gridK;
    const int* gridJ;
    bool hasGrid;
    int passIndex;

    ScanFrame() : ipoc(0), si(0), beam(49), worldXYZ(nullptr), worldCount(0), hasWorld(false), gridK(nullptr), gridJ(nullptr), hasGrid(false), passIndex(0) {
        memset(amp, 0, sizeof(amp));
        memset(tof, 0, sizeof(tof));
        memset(beamZ, 0, sizeof(beamZ));
        memset(pose, 0, sizeof(pose));
    }
};
Q_DECLARE_METATYPE(ScanFrame)

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

    // 停止数据采集
    void stop();

    // 暂停数据采集（保留位置，可继续）
    void pause();

    // 判断是否已加载文件
    bool isFileLoaded() const;
    bool isGridReady() const { return m_gridReady; }
    // Compensate a fixed time offset between the AMP/TOF channel and the
    // robot pose. Positive: AMP/TOF lags the pose (pose is delayed by N
    // frames to meet it). Negative: AMP/TOF leads the pose (AMP/TOF is
    // delayed by |N| frames).
    void setUsDelayFrames(int frames) { m_usDelayFrames = frames; }
    void setOnlineGridEnabled(bool on) { m_onlineGridEnabled = on; }
    bool isOnlineGridEnabled() const { return m_onlineGridEnabled; }
    // 是否保存扫描数据（实时模式默认开启；回放模式不写盘）
    void setSaveCsvEnabled(bool on) { m_saveCsv = on; }

    // 离线加载：在 Scan 线程内用与回放完全相同的在线网格管线处理整个
    // CSV，完成后通过 buildFinished 通知；结果可用 loadedFrames() 读取
    // （ScanFrame::worldXYZ 指向内部网格数据，消费完之前不要再次 load/start）。
    void loadCSVAndBuild(const QString& filename);
    void cancelBuild();
    const std::vector<ScanFrame>& loadedFrames() const { return m_loadedFrames; }

    // 完全重置扫描状态（停止采集、清除已加载文件/网格/暂停状态），
    // 供“重置数据”按钮使用；必须在 Scan 线程内调用。
    void resetForNewScan();

signals:
    void newFrameAvailable(const ScanFrame& frame);
    void finished();
    void buildProgress(int done, int total);
    void buildFinished(bool ok, int frameCount);

private slots:
    void sendNextFrame();

private:
    std::vector<std::string> parseCSVLine(const std::string& line);
    double safe_stod(const std::string& str, double defaultVal = 0.0);
    void updateGlobalVariables(const std::vector<std::string>& cells);
    void computeBeamDepths();
    void computeBeamDepthOnline();
    void buildUniformCloud();
    // 数据落盘（Scan 线程内统一写入，避免多线程各自采样全局 IPOC）
    void openCsvSave();
    void writeCsvRow();
    void flushCsvBuf();
    void closeCsvSave();
    struct FrameRecord;
    struct PassFrameMeta;
    void processFrameOnline(const FrameRecord& rec, const PassFrameMeta& meta, int dir);
    void emitGridCell(long long k, long long j, float z, float a, float t,
                      double ux, double uy, double vx2, double vy2);
    void flushOnlinePack();
    void flushOnlineTail();
    void insertRowCell(long long k, long long j, float z, float a, float t, int e,
                       double ux, double uy, double vx2, double vy2);
    void finalizeFilteredRow(long long j);
    void finalizeAllRows();

private:
    std::vector<std::vector<std::string>> m_data;
    std::string m_csvPath;
    // 宽字符路径：Windows 下 std::ifstream 的窄字符串重载按 ANSI 代码页
    // 转换路径（本机 GBK），UTF-8 字节的中文路径会打不开，导致加载/回放
    // 0 帧。文件流统一用宽字符路径打开。
    std::wstring m_csvPathW;
    std::ifstream m_dataStream;
    std::vector<std::array<float, 64>> m_beamDepths;
    std::vector<float> m_fitCx, m_fitCy, m_fitCz, m_fitSi;
    std::unordered_map<long long, std::vector<int>> m_fitGrid;
    int m_lastValidFit = -1;
    float m_lastValidSi = 0.0f;
    int m_usDelayFrames = 0;
    struct DelayedUs {
        double pose[6];
        float amp[64];
        float tof[64];
        int frameIdx;
    };
    std::deque<DelayedUs> m_usRing;
    bool m_onlineGridEnabled = true;
    struct FrameRecord {
        float xyz[49*3];
        float amp[49];
        float tof[49];
    };
    std::deque<FrameRecord> m_frameRing;
    double m_ugx = 0, m_ugy = 0, m_ugz = 0;
    int m_latCount = 0;
    struct OnlineCell {
        float z = 0, amp = 0, tof = 0;
    };
    std::unordered_map<long long, OnlineCell> m_onlineCells;
    std::deque<GridFrame> m_onlineOut;
    GridFrame m_onlineGf;
    int m_onlinePacked = 0;
    int m_onlineIndex = 0;
    std::deque<double> m_uHist, m_sHist, m_zHist;
    // 实时模式：以上一帧 IPOC 判断是否有新的硬件帧（机器人 4ms/250Hz）
    quint32 m_lastIpoc = 0;
    quint64 m_lastIpocChangeMs = 0;   // 最近一次 IPOC 变化的时间（停滞检测用）
    int m_passDir = 0;
    int m_passFrames = 0;
    double m_passUsum = 0;
    int m_passUcnt = 0;
    double m_prevPassCenter = 0;
    bool m_prevPassCenterValid = false;
    double m_lastScS = 0;
    bool m_lastScSValid = false;
    double m_curShift = 0;
    int m_passStepSign = 0;
    struct PassFrameMeta {
        double uc0, ucS, sc0, scS, zc0, zcS;
        bool turn;
    };
    struct CandCell { long long k, j; float z, a, t; double d2; int e = -1; };
    bool m_pass0RefFrozen = false;
    struct RowCell { float z, a, t; int e = -1; };
    struct RowBuf {
        std::map<long long, RowCell> cells;  // k -> cell
        bool done = false;
    };
    std::map<long long, RowBuf> m_rowBuf;    // j -> row
    bool m_rowBufActive = false;
    bool m_bandDirFwd = true;
    long long m_nextFinalizeJ = 0;
    int m_filterWin = 1;                     // 2D ?????5x5?
    long long m_noiseSuppressed = 0;
    double m_pass0Ref = 0;
    int m_passIndex = 0;
    int m_candDir = 0;
    int m_candDirCnt = 0;
    int m_lastFlipFrame = -100000;
    bool m_paused = false;
    // 软暂停：实时模式下暂停命令到达后先继续轮询，等机器人 IPOC 真正
    // 停稳（约 800ms）再停表，避免暂停命令到机器人停住之间的滑行段丢帧。
    bool m_pausePending = false;
    quint32 m_pauseLastIpoc = 0;
    QElapsedTimer m_pauseStableTimer;
    bool m_pauseStableValid = false;
    std::deque<std::unordered_map<long long, CandCell>> m_win;
    int m_winMax = 6;
    std::deque<FrameRecord> m_colorHist;
    std::vector<GridFrame> m_gridCloud;   // 方案2：均匀网格重采样点云
    bool m_gridReady = false;
    int m_gridIndex = 0;
    bool m_beamDepthsReady = false;
    std::map<std::string, int> m_columnMap;
    int m_currentIndex;
    int m_totalFrames;

    // 列索引
    int m_colBEAM, m_colLX, m_colLY, m_colX, m_colSI, m_colAMP_1, m_colTOF_1;

    QTimer* m_timer;
    bool m_isRunning;
    double m_lastLX = 0.0;
    double m_lastLY = 0.0;


    // ============ 数据保存（CSV） ============
    bool m_saveCsv = true;          // 实时模式保存数据
    QFile m_saveCsvFile;            // 输出文件（Unicode 路径安全）
    QByteArray m_saveCsvBuf;        // 行缓冲，攒批写盘
    QString m_saveCsvPath;          // 输出文件完整路径
    bool m_saveCsvOpen = false;
    qint64 m_csvRows = 0;           // 已写行数（表头不计）
    quint32 m_lastSavedIpoc = 0;    // 已写行对应的 IPOC（连续性检查）
    bool m_lastSavedIpocValid = false;
    QElapsedTimer m_csv10sTimer;    // 10s 帧率统计
    bool m_csv10sValid = false;
    int m_csv10sFrames = 0;

    // ============ 实时缺帧统计 ============
    quint64 m_lostFrames = 0;       // IPOC 跳变累计丢失帧

    // ============ 离线加载（loadCSVAndBuild）============
    bool m_suppressEmit = false;            // 离线构建期间禁止发 newFrameAvailable
    bool m_offlineBuild = false;            // 离线构建中（EOF 分支不发 finished）
    std::atomic<bool> m_buildCancel{false}; // 用户取消离线构建
    std::vector<ScanFrame> m_loadedFrames;  // 离线构建结果（指向 m_onlineOut 网格）
};

#endif // SCAN_H
