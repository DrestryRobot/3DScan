// Scan.cpp
#include "Scan.h"
#include "scandata.h"
#include <QDebug>
#include <QFile>
#include <QDateTime>
#include <QTextStream>
#include <QElapsedTimer>
#include <QDir>
#include <cstdio>
#include <unordered_map>
#include <cmath>
#include <algorithm>

// ============================================
// 全局变量定义统一放在 scandata.cpp（scandata.h 提供 extern 声明）
// ============================================

// ============================================
// Frame offset between the AMP/TOF channel and the robot pose.
//   > 0 : AMP/TOF is N frames SLOWER than the pose (pose is delayed by N)
//   < 0 : AMP/TOF is N frames FASTER than the pose (AMP/TOF is delayed by |N|)
// Change this number and rebuild.
// ============================================
static const int kUsDelayFrames = -8;
// 小噪点抑制：保留≥1mm 缺陷（约 ≥3 格），去除 0.3~0.6mm 孤立噪点。
// 当前值偏离 3x3 局部中位数超过阈值且同向偏离的邻居 < kMinSupport 个时，判为孤立噪点压回背景。
static const float kAmpNoiseDev = 0.08f;
static const float kTofNoiseDev = 0.15f;
static const int   kMinSupport = 2;


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
    m_usDelayFrames = kUsDelayFrames;
    m_timer = new QTimer(this);
    m_timer->setTimerType(Qt::PreciseTimer);
    connect(m_timer, &QTimer::timeout, this, &Scan::sendNextFrame);
}

Scan::~Scan()
{
    // timer is stopped in the worker thread before Scan is destroyed
    closeCsvSave();
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
    m_csvPath = filename.toStdString();
    m_csvPathW = filename.toStdWString();
    m_dataStream.close();
    m_data.clear();
    m_columnMap.clear();
    m_beamDepths.clear();
    m_beamDepthsReady = false;
    m_fitCx.clear(); m_fitCy.clear(); m_fitCz.clear(); m_fitSi.clear();
    m_fitGrid.clear();
    m_lastValidFit = -1;
    m_lastValidSi = 0.0f;
    m_onlineCells.clear();
    m_onlineOut.clear();
    m_loadedFrames.clear();
    m_onlineGf = GridFrame();
    m_onlinePacked = 0;
    m_onlineIndex = 0;
    m_uHist.clear(); m_sHist.clear(); m_zHist.clear();
    m_frameRing.clear();
    m_latCount = 0;
    m_ugx = m_ugy = m_ugz = 0;
    m_passDir = 0;
    m_passFrames = 0;
    m_passUsum = 0;
    m_passUcnt = 0;
    m_prevPassCenter = 0;
    m_prevPassCenterValid = false;
    m_lastScS = 0;
    m_lastScSValid = false;
    m_curShift = 0;
    m_passStepSign = 0;
    m_pass0RefFrozen = false;
    m_pass0Ref = 0;
    m_passIndex = 0;
    m_candDir = 0;
    m_candDirCnt = 0;
    m_lastFlipFrame = -100000;
    m_paused = false;
    m_rowBuf.clear();
    m_rowBufActive = false;
    m_bandDirFwd = true;
    m_nextFinalizeJ = 0;
    m_noiseSuppressed = 0;
    m_win.clear();
    m_colorHist.clear();
    m_lastLX = 0.0;
    m_lastLY = 0.0;

    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Cannot open file:" << filename;
        return false;
    }

    QTextStream stream(&file);
    QString headerLine = stream.readLine();
    if (headerLine.isEmpty()) {
        file.close();
        return false;
    }
    file.close();

    std::vector<std::string> headers = parseCSVLine(headerLine.toStdString());
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

    m_totalFrames = 0;
    m_currentIndex = 0;

    qDebug() << "CSV source ready for 250Hz real-time streaming";
    return true;
}

void Scan::computeBeamDepths()
{
    m_beamDepths.clear();
    m_beamDepthsReady = false;

    const int N = (int)m_data.size();
    if (N <= 0) return;

    struct Vec3 { double x, y, z; };
    struct Mat3 { double m[3][3]; };

    auto makeRotation = [](double A, double B, double C, Mat3& R) {
        const double d2r = 0.017453292519943295;
        double a = A * d2r, b = B * d2r, c = C * d2r;
        double ca = cos(a), sa = sin(a);
        double cb = cos(b), sb = sin(b);
        double cc = cos(c), sc = sin(c);
        // KUKA: R = Rz(A) * Ry(B) * Rx(C)
        R.m[0][0] = ca*cb;                 R.m[0][1] = ca*sb*sc - sa*cc; R.m[0][2] = ca*sb*cc + sa*sc;
        R.m[1][0] = sa*cb;                 R.m[1][1] = sa*sb*sc + ca*cc; R.m[1][2] = sa*sb*cc - ca*sc;
        R.m[2][0] = -sb;                   R.m[2][1] = cb*sc;            R.m[2][2] = cb*cc;
    };

    std::vector<double> Tx(N), Ty(N), Tz(N), siV(N);
    std::vector<Mat3> Rm(N);
    std::vector<Vec3> P(N);

    double lastLX = 0.0, lastLY = 0.0;
    for (int i = 0; i < N; i++) {
        const std::vector<std::string>& cells = m_data[i];
        if (m_colLX != -1 && m_colLX < (int)cells.size() && !cells[m_colLX].empty())
            lastLX = safe_stod(cells[m_colLX], lastLX);
        if (m_colLY != -1 && m_colLY < (int)cells.size() && !cells[m_colLY].empty())
            lastLY = safe_stod(cells[m_colLY], lastLY);

        double pose[6] = {0, 0, 0, 0, 0, 0};
        if (m_colX != -1) {
            for (int k = 0; k < 6 && (m_colX + k) < (int)cells.size(); k++)
                pose[k] = safe_stod(cells[m_colX + k], 0.0);
        }
        pose[0] += lastLY;
        pose[1] -= lastLX;

        double siVal = 0.0;
        if (m_colSI != -1 && m_colSI < (int)cells.size())
            siVal = safe_stod(cells[m_colSI], 0.0);

        Tx[i] = pose[0]; Ty[i] = pose[1]; Tz[i] = pose[2]; siV[i] = siVal;
        makeRotation(pose[3], pose[4], pose[5], Rm[i]);

        double local[3] = {0.0, 0.0, -siVal * 0.5};
        P[i].x = pose[0] + Rm[i].m[0][0]*local[0] + Rm[i].m[0][1]*local[1] + Rm[i].m[0][2]*local[2];
        P[i].y = pose[1] + Rm[i].m[1][0]*local[0] + Rm[i].m[1][1]*local[1] + Rm[i].m[1][2]*local[2];
        P[i].z = pose[2] + Rm[i].m[2][0]*local[0] + Rm[i].m[2][1]*local[1] + Rm[i].m[2][2]*local[2];
    }

    // Spatial hash over the center points (world XY plane).
    const double cell = 12.0;
    double minx = P[0].x, miny = P[0].y;
    for (int i = 1; i < N; i++) {
        if (P[i].x < minx) minx = P[i].x;
        if (P[i].y < miny) miny = P[i].y;
    }

    auto cellKey = [](int gx, int gy) -> long long {
        return ((long long)(gx + 100000) << 32) | (unsigned int)(gy + 100000);
    };
    std::unordered_map<long long, std::vector<int>> grid;
    for (int i = 0; i < N; i++) {
        int gx = (int)floor((P[i].x - minx) / cell);
        int gy = (int)floor((P[i].y - miny) / cell);
        grid[cellKey(gx, gy)].push_back(i);
    }

    m_beamDepths.resize(N);
    std::vector<char> fitValid(N, 0);
    const int centerBeam = 24; // 49 elements -> center index 24
    const double spacing = 0.3;
    const int maxBeam = 64;

    std::vector<double> xs, ys, zs, ws;
    xs.reserve(700); ys.reserve(700); zs.reserve(700); ws.reserve(700);

    for (int i = 0; i < N; i++) {
        // Fallback: keep the original uniform SI depth for every element.
        for (int j = 0; j < maxBeam; j++)
            m_beamDepths[i][j] = (float)(-siV[i] * 0.5);

        int gx = (int)floor((P[i].x - minx) / cell);
        int gy = (int)floor((P[i].y - miny) / cell);

        xs.clear(); ys.clear(); zs.clear(); ws.clear();
        double yMin = 1e300, yMax = -1e300;
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -3; dy <= 3; dy++) {
                auto it = grid.find(cellKey(gx + dx, gy + dy));
                if (it == grid.end()) continue;
                const std::vector<int>& ids = it->second;
                for (int idx : ids) {
                    double wx = P[idx].x - Tx[i];
                    double wy = P[idx].y - Ty[i];
                    double wz = P[idx].z - Tz[i];
                    double xl = wx*Rm[i].m[0][0] + wy*Rm[i].m[1][0] + wz*Rm[i].m[2][0];
                    double yl = wx*Rm[i].m[0][1] + wy*Rm[i].m[1][1] + wz*Rm[i].m[2][1];
                    double zl = wx*Rm[i].m[0][2] + wy*Rm[i].m[1][2] + wz*Rm[i].m[2][2];
                    if (fabs(xl) > 12.0 || fabs(yl) > 30.0) continue;
                    xs.push_back(xl); ys.push_back(yl); zs.push_back(zl);
                    ws.push_back(exp(-(xl*xl + yl*yl) / 100.0));
                    if (yl < yMin) yMin = yl;
                    if (yl > yMax) yMax = yl;
                }
            }
        }
        int n = (int)xs.size();
        if (n < 30 || (yMax - yMin) < 8.0) continue;

        // Weighted least squares with a constant term:
        //   h(x,y) = c0 + c1*x + c2*y + c3*x*y + c4*x^2 + c5*y^2
        // c0 lets the local fit correct an outlying SI center point.
        double ATA[6][6] = {{0}};
        double ATb[6] = {0};
        for (int s = 0; s < n; s++) {
            double x = xs[s], y = ys[s], z = zs[s], w = ws[s];
            double h = z + siV[i] * 0.5;
            double f[6] = {w, x*w, y*w, x*y*w, x*x*w, y*y*w};
            double b = h * w;
            for (int a = 0; a < 6; a++) {
                ATb[a] += f[a] * b;
                for (int c2 = 0; c2 < 6; c2++)
                    ATA[a][c2] += f[a] * f[c2];
            }
        }
        for (int a = 0; a < 6; a++) ATA[a][a] += 1e-9;

        // Solve the 6x6 normal equations (Gaussian elimination with pivoting).
        double M[6][7];
        for (int a = 0; a < 6; a++) {
            for (int c2 = 0; c2 < 6; c2++) M[a][c2] = ATA[a][c2];
            M[a][6] = ATb[a];
        }
        bool ok = true;
        for (int col = 0; col < 6; col++) {
            int piv = col;
            for (int r = col + 1; r < 6; r++)
                if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
            if (fabs(M[piv][col]) < 1e-30) { ok = false; break; }
            if (piv != col)
                for (int c2 = col; c2 < 7; c2++) std::swap(M[piv][c2], M[col][c2]);
            for (int r = 0; r < 6; r++) {
                if (r == col) continue;
                double factor = M[r][col] / M[col][col];
                for (int c2 = col; c2 < 7; c2++) M[r][c2] -= factor * M[col][c2];
            }
        }
        if (!ok) continue;
        double coef[6];
        for (int r = 0; r < 6; r++) coef[r] = M[r][6] / M[r][r];

        // Lateral curvature is trusted only when the sampled neighborhood
        // extends to both sides; one-sided edges fade the lateral correction.
        double symv = (yMax < -yMin) ? yMax : -yMin;
        double wsym = symv / 10.0;
        if (wsym < 0.0) wsym = 0.0;
        if (wsym > 1.0) wsym = 1.0;

        for (int j = 0; j < maxBeam; j++) {
            double y = (j - centerBeam) * spacing;
            if (y < yMin) y = yMin;
            if (y > yMax) y = yMax;
            double hc = coef[0] + wsym * (coef[2] * y + coef[5] * y * y);
            if (hc > 5.0) hc = 5.0;
            if (hc < -5.0) hc = -5.0;
            m_beamDepths[i][j] = (float)(-siV[i] * 0.5 + hc);
        }
        fitValid[i] = 1;
    }

    // Second pass: edge frames with no usable lateral neighborhood inherit the
    // curvature from the nearest valid frame (in time), keeping the center depth
    // from their own SI so the transition stays continuous.
    const int maxSearch = 20000;
    const double edgeBlendFrames = 100.0;
    for (int i = 0; i < N; i++) {
        if (fitValid[i]) continue;
        int best = -1;
        int dist = 0;
        for (int step = 1; step <= maxSearch; step++) {
            int back = i - step;
            int fwd = i + step;
            if (back >= 0 && fitValid[back]) { best = back; dist = step; break; }
            if (fwd < N && fitValid[fwd]) { best = fwd; dist = step; break; }
        }
        if (best < 0) continue;
        double alpha = 1.0 - (double)dist / edgeBlendFrames;
        if (alpha <= 0.0) continue;
        double siBest = siV[best];
        double uniform = -siV[i] * 0.5;
        for (int j = 0; j < maxBeam; j++) {
            double hc = (double)m_beamDepths[best][j] + siBest * 0.5;
            if (hc > 5.0) hc = 5.0;
            if (hc < -5.0) hc = -5.0;
            m_beamDepths[i][j] = (float)(uniform + alpha * hc);
        }
    }

    m_beamDepthsReady = true;
}
void Scan::buildUniformCloud()
{
    m_gridCloud.clear();
    m_gridReady = false;
    m_gridIndex = 0;

    const int N = (int)m_data.size();
    if (N <= 0 || !m_beamDepthsReady || (int)m_beamDepths.size() < N) {
        qWarning() << "buildUniformCloud: beam depths not ready";
        return;
    }

    const double d2r = 0.017453292519943295;
    const int centerBeam = 24;
    const double elemPitch = 0.3;
    const int maxBeam = 49;
    const double deltaS = 0.55;

    struct V3 { double x, y, z; };

    auto makeRotation = [d2r](double A, double B, double C, double R[3][3]) {
        double a = A*d2r, b = B*d2r, c = C*d2r;
        double ca=cos(a), sa=sin(a), cb=cos(b), sb=sin(b), cc=cos(c), sc=sin(c);
        R[0][0]=ca*cb; R[0][1]=ca*sb*sc-sa*cc; R[0][2]=ca*sb*cc+sa*sc;
        R[1][0]=sa*cb; R[1][1]=sa*sb*sc+ca*cc; R[1][2]=sa*sb*cc-ca*sc;
        R[2][0]=-sb;   R[2][1]=cb*sc;          R[2][2]=cb*cc;
    };

    // per-frame center points, probe array direction, velocity
    std::vector<V3> P(N), lat(N), vel(N);
    std::vector<double> siV(N);
    std::vector<double> poseAll(6 * N);
    double lastLX = 0.0, lastLY = 0.0;
    for (int i = 0; i < N; i++) {
        const std::vector<std::string>& cells = m_data[i];
        if (m_colLX != -1 && m_colLX < (int)cells.size() && !cells[m_colLX].empty())
            lastLX = safe_stod(cells[m_colLX], lastLX);
        if (m_colLY != -1 && m_colLY < (int)cells.size() && !cells[m_colLY].empty())
            lastLY = safe_stod(cells[m_colLY], lastLY);

        double pose[6] = {0,0,0,0,0,0};
        if (m_colX != -1)
            for (int k = 0; k < 6 && (m_colX+k) < (int)cells.size(); k++)
                pose[k] = safe_stod(cells[m_colX+k], 0.0);
        pose[0] += lastLY;
        pose[1] -= lastLX;
        memcpy(&poseAll[(size_t)i*6], pose, 6*sizeof(double));

        siV[i] = (m_colSI != -1 && m_colSI < (int)cells.size())
                     ? safe_stod(cells[m_colSI], 0.0) : 0.0;

        double R[3][3];
        makeRotation(pose[3], pose[4], pose[5], R);
        double zc = m_beamDepths[i][centerBeam];
        P[i].x = pose[0] + R[0][2]*zc;
        P[i].y = pose[1] + R[1][2]*zc;
        P[i].z = pose[2] + R[2][2]*zc;

        double lx = R[0][1], ly = R[1][1], lz = R[2][1];
        double ln = sqrt(lx*lx + ly*ly + lz*lz);
        if (ln < 1e-12) ln = 1.0;
        lat[i].x = lx/ln; lat[i].y = ly/ln; lat[i].z = lz/ln;
    }
    for (int i = 0; i < N; i++) {
        int ia = (i == 0) ? 0 : i-1;
        int ib = (i == N-1) ? N-1 : i+1;
        vel[i].x = (P[ib].x - P[ia].x) * 0.5;
        vel[i].y = (P[ib].y - P[ia].y) * 0.5;
        vel[i].z = (P[ib].z - P[ia].z) * 0.5;
    }

    // projection of velocity on pass direction (perpendicular to array in XY)
    std::vector<double> proj(N);
    for (int i = 0; i < N; i++) {
        double px = -lat[i].y, py = lat[i].x;
        double pn = sqrt(px*px + py*py);
        if (pn < 1e-12) pn = 1.0;
        px /= pn; py /= pn;
        proj[i] = vel[i].x*px + vel[i].y*py;
    }
    const int win = 31, half = win/2;
    std::vector<double> avg(N);
    for (int i = 0; i < N; i++) {
        double sum = 0.0;
        for (int d = -half; d <= half; d++) {
            int j = i + d;
            if (j < 0) j = 0;
            if (j >= N) j = N-1;
            sum += proj[j];
        }
        avg[i] = sum / win;
    }

    struct PassSeg { int a, b; double uc; };
    std::vector<PassSeg> passes;
    {
        int start = 0;
        int cur = (avg[0] >= 0.0) ? 1 : -1;
        for (int i = 1; i <= N; i++) {
            int s = (i < N) ? ((avg[i] >= 0.0) ? 1 : -1) : -cur;
            if (i == N || s != cur) {
                if (i - start >= 100) {
                    std::vector<double> ap(i - start);
                    for (int k = start; k < i; k++) ap[k-start] = fabs(proj[k]);
                    std::sort(ap.begin(), ap.end());
                    double med = ap[ap.size()/2];
                    if (med >= 0.3)
                        passes.push_back({start, i, 0.0});
                }
                start = i;
                cur = s;
            }
        }
    }
    if (passes.size() < 2) {
        qWarning() << "buildUniformCloud: too few passes" << passes.size();
        return;
    }

    // global lateral direction and per-pass lateral coordinate
    V3 ug = {0,0,0};
    for (int i = 0; i < N; i++) { ug.x += lat[i].x; ug.y += lat[i].y; ug.z += lat[i].z; }
    double un = sqrt(ug.x*ug.x + ug.y*ug.y + ug.z*ug.z);
    if (un < 1e-12) return;
    ug.x /= un; ug.y /= un; ug.z /= un;

    for (size_t p = 0; p < passes.size(); p++) {
        std::vector<double> uc;
        uc.reserve(passes[p].b - passes[p].a);
        for (int i = passes[p].a; i < passes[p].b; i++)
            uc.push_back(P[i].x*ug.x + P[i].y*ug.y + P[i].z*ug.z);
        std::sort(uc.begin(), uc.end());
        passes[p].uc = uc[uc.size()/2];
    }
    std::vector<int> order(passes.size());
    for (size_t i = 0; i < order.size(); i++) order[i] = (int)i;
    std::sort(order.begin(), order.end(),
              [&](int a, int b){ return passes[a].uc < passes[b].uc; });

    std::vector<double> uLo(passes.size()), uHi(passes.size());
    for (size_t q = 0; q < order.size(); q++) {
        int p = order[q];
        double lo = (q == 0) ? passes[p].uc - 7.2
                             : 0.5 * (passes[order[q-1]].uc + passes[p].uc);
        double hi = (q == order.size()-1) ? passes[p].uc + 7.2
                                          : 0.5 * (passes[p].uc + passes[order[q+1]].uc);
        uLo[p] = lo;
        uHi[p] = hi;
    }

    // per-pass buffers
    std::vector<double> s;
    std::vector<float> W;
    std::vector<double> poseF;
    std::vector<float> ampF, tofF;
    for (size_t p = 0; p < passes.size(); p++) {
        int a = passes[p].a, b = passes[p].b;
        int M = b - a;

        s.resize(M);
        s[0] = 0.0;
        for (int i = 1; i < M; i++) {
            double dx = P[a+i].x - P[a+i-1].x;
            double dy = P[a+i].y - P[a+i-1].y;
            double dz = P[a+i].z - P[a+i-1].z;
            s[i] = s[i-1] + sqrt(dx*dx + dy*dy + dz*dz);
            if (s[i] < s[i-1]) s[i] = s[i-1];
        }

        W.resize((size_t)M * maxBeam * 3);
        poseF.resize((size_t)M * 6);
        ampF.resize((size_t)M * maxBeam);
        tofF.resize((size_t)M * maxBeam);

        for (int i = 0; i < M; i++) {
            const double* pose = &poseAll[(size_t)(a+i)*6];
            double R[3][3];
            makeRotation(pose[3], pose[4], pose[5], R);
            for (int e = 0; e < maxBeam; e++) {
                double y = (e - centerBeam) * elemPitch;
                double z = m_beamDepths[a+i][e];
                size_t o = (size_t)(i*maxBeam + e) * 3;
                W[o+0] = (float)(pose[0] + R[0][1]*y + R[0][2]*z);
                W[o+1] = (float)(pose[1] + R[1][1]*y + R[1][2]*z);
                W[o+2] = (float)(pose[2] + R[2][1]*y + R[2][2]*z);

                const std::vector<std::string>& cells = m_data[a+i];
                int ampIdx = (m_colAMP_1 != -1) ? m_colAMP_1 + 2*e : -1;
                int tofIdx = (m_colTOF_1 != -1) ? m_colTOF_1 + 2*e : -1;
                ampF[(size_t)i*maxBeam + e] =
                    (ampIdx >= 0 && ampIdx < (int)cells.size())
                        ? (float)safe_stod(cells[ampIdx], 0.0) : 0.0f;
                tofF[(size_t)i*maxBeam + e] =
                    (tofIdx >= 0 && tofIdx < (int)cells.size())
                        ? (float)safe_stod(cells[tofIdx], 0.0) : 0.0f;
            }
            memcpy(&poseF[(size_t)i*6], pose, 6*sizeof(double));
        }

        double ucP = passes[p].uc;
        double w = uHi[p] - uLo[p];
        if (w < 1e-6) w = 14.4;
        int rows = (int)(s[M-1] / deltaS) + 1;
        if (rows <= 0) continue;

        for (int j = 0; j < rows; j++) {
            double sj = j * deltaS;
            int lo = 0, hi = M-1;
            while (lo < hi) {
                int mid = (lo + hi + 1) / 2;
                if (s[mid] <= sj) lo = mid; else hi = mid-1;
            }
            int i0 = lo, i1 = lo + 1;
            if (i1 >= M) i1 = M-1;
            double t = (s[i1] > s[i0]) ? (sj - s[i0]) / (s[i1] - s[i0]) : 0.0;

            GridFrame gf;
            gf.si = (float)(siV[a+i0]*(1.0-t) + siV[a+i1]*t);
            for (int k = 0; k < 6; k++)
                gf.pose[k] = poseF[(size_t)i0*6+k]*(1.0-t) + poseF[(size_t)i1*6+k]*t;

            for (int k = 0; k < maxBeam; k++) {
                double uk = uLo[p] + ((double)k + 0.5) * w / maxBeam;
                double r = uk - ucP;
                double fe = (r + 7.2) / elemPitch;
                int e0 = (int)floor(fe);
                if (e0 < 0) e0 = 0;
                if (e0 > maxBeam-2) e0 = maxBeam-2;
                double te = fe - e0;
                if (te < 0.0) te = 0.0;
                if (te > 1.0) te = 1.0;

                size_t o0 = (size_t)i0*maxBeam + e0;
                size_t o1 = (size_t)i1*maxBeam + e0;
                double x0 = W[o0*3+0]*(1.0-te) + W[o0*3+3]*te;
                double y0 = W[o0*3+1]*(1.0-te) + W[o0*3+4]*te;
                double z0 = W[o0*3+2]*(1.0-te) + W[o0*3+5]*te;
                double x1 = W[o1*3+0]*(1.0-te) + W[o1*3+3]*te;
                double y1 = W[o1*3+1]*(1.0-te) + W[o1*3+4]*te;
                double z1 = W[o1*3+2]*(1.0-te) + W[o1*3+5]*te;

                double a0 = ampF[o0]*(1.0-te) + ampF[o0+1]*te;
                double t0 = tofF[o0]*(1.0-te) + tofF[o0+1]*te;
                double a1 = ampF[o1]*(1.0-te) + ampF[o1+1]*te;
                double t1 = tofF[o1]*(1.0-te) + tofF[o1+1]*te;

                gf.pts[k].x = (float)(x0*(1.0-t) + x1*t);
                gf.pts[k].y = (float)(y0*(1.0-t) + y1*t);
                gf.pts[k].z = (float)(z0*(1.0-t) + z1*t);
                gf.amp[k] = (float)(a0*(1.0-t) + a1*t);
                gf.tof[k] = (float)(t0*(1.0-t) + t1*t);
            }
            gf.valid = maxBeam;
            m_gridCloud.push_back(gf);
        }
    }

    m_gridReady = true;
}

void Scan::updateGlobalVariables(const std::vector<std::string>& cells)
{


    // 读取波束
    int beamVal = 49;
    if (m_colBEAM != -1 && m_colBEAM < (int)cells.size()) {
        beamVal = static_cast<int>(safe_stod(cells[m_colBEAM], 49));
    }
    beam = (beamVal > 0 && beamVal <= 64) ? beamVal : 49;

    // 读取龙门坐标
    if (m_colLX != -1 && m_colLX < (int)cells.size() && !cells[m_colLX].empty()) {
        m_lastLX = safe_stod(cells[m_colLX], m_lastLX);
    }
    if (m_colLY != -1 && m_colLY < (int)cells.size() && !cells[m_colLY].empty()) {
        m_lastLY = safe_stod(cells[m_colLY], m_lastLY);
    }
    longmen[0] = m_lastLX;
    longmen[1] = m_lastLY;

    // 读取机器人位姿
    double pose[6] = {0, 0, 0, 0, 0, 0};
    if (m_colX != -1) {
        for (int i = 0; i < 6 && (m_colX + i) < (int)cells.size(); i++) {
            pose[i] = safe_stod(cells[m_colX + i], 0.0);
        }
    }
    robot_x = pose[0] + m_lastLY;
    robot_y = pose[1] - m_lastLX;
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
            beamValid[i] = true;
        }
    }

    // 数据更新标志
    robot_ipoc++;
}

// ============================================
// 数据保存（CSV）：Scan 线程内统一落盘，
// 不再由 SoundScan 独立线程采样全局量（避免双源竞争和采样丢帧误报）
// ============================================
void Scan::openCsvSave()
{
    if (m_saveCsvOpen) return;

    QDir dir("C:/超声扫描/报告");
    if (!dir.exists()) dir.mkpath(".");

    QString base = QString("scan_%1").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString full = dir.filePath(base + ".csv");
    int n = 1;
    while (QFile::exists(full)) {
        full = dir.filePath(QString("%1_%2.csv").arg(base).arg(n++));
    }

    m_saveCsvFile.setFileName(full);
    if (!m_saveCsvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qDebug() << "[Scan] 无法创建CSV文件:" << full;
        return;
    }

    m_saveCsvPath = full;
    m_saveCsvOpen = true;
    m_csvRows = 0;
    m_lastSavedIpocValid = false;
    m_csv10sFrames = 0;
    m_csv10sValid = true;
    m_csv10sTimer.start();

    // 表头：X,Y,Z,A,B,C,SI,AMP_1..49,TOF_1..49,BEAM,LX,LY
    QByteArray hdr = QByteArrayLiteral("X,Y,Z,A,B,C,SI,");
    int nb = beam; if (nb > 64) nb = 64;
    for (int i = 1; i <= nb; i++) {
        hdr += "AMP_" + QByteArray::number(i) + ",TOF_" + QByteArray::number(i) + ",";
    }
    hdr += QByteArrayLiteral("BEAM,LX,LY\n");
    m_saveCsvBuf += hdr;
    flushCsvBuf();

    qDebug() << "[Scan] CSV文件已创建:" << full;
}

void Scan::writeCsvRow()
{
    if (!m_saveCsvOpen) return;

    // 保存路径自身的 IPOC 连续性（CSV 按 250Hz 机器人帧写行，
    // 差值 >4 即丢失了机器人帧，属真实丢帧）
    if (m_lastSavedIpocValid) {
        quint32 delta = robot_ipoc - m_lastSavedIpoc;
        if (delta > 4) {
            int lost = (int)(delta / 4) - 1;
            qDebug() << "[IPOC不连续] 上次:" << m_lastSavedIpoc
                     << "当前:" << robot_ipoc
                     << "差值:" << delta
                     << "丢失" << lost << "帧"
                     << "行号:" << m_csvRows;
        }
    }
    m_lastSavedIpoc = robot_ipoc;
    m_lastSavedIpocValid = true;

    // 10s 帧率统计
    m_csv10sFrames++;
    if (m_csv10sValid && m_csv10sTimer.elapsed() >= 10000) {
        m_csv10sTimer.restart();
        m_csv10sFrames = 0;
    }

    m_csvRows++;

    char buf[4096];
    int off = 0;
    off += snprintf(buf + off, sizeof(buf) - off,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
                    robot_x, robot_y, robot_z, robot_a, robot_b, robot_c, si);
    int nb = beam; if (nb > 64) nb = 64;
    for (int i = 0; i < nb; i++) {
        off += snprintf(buf + off, sizeof(buf) - off, "%.6f,%.6f,", amp[i], tof[i]);
    }
    // 与旧格式保持一致：仅第一行写 BEAM/LX/LY（龙门坐标取全局 longmen，ADS 实时更新）
    if (m_csvRows == 1) {
        off += snprintf(buf + off, sizeof(buf) - off, "%d,%.6f,%.6f\n",
                        beam, longmen[0], longmen[1]);
    } else {
        off += snprintf(buf + off, sizeof(buf) - off, ",,,\n");
    }
    m_saveCsvBuf.append(buf, off);

    // 攒批写盘（约每 1000 行一次）
    if (m_csvRows % 1000 == 0) flushCsvBuf();
}

void Scan::flushCsvBuf()
{
    if (!m_saveCsvOpen || m_saveCsvBuf.isEmpty()) return;
    m_saveCsvFile.write(m_saveCsvBuf);
    m_saveCsvBuf.clear();
}

void Scan::closeCsvSave()
{
    if (!m_saveCsvOpen) return;
    flushCsvBuf();
    m_saveCsvFile.flush();
    m_saveCsvFile.close();
    m_saveCsvOpen = false;
    qDebug() << "[Scan] CSV文件已关闭，共写入" << m_csvRows << "行 ->" << m_saveCsvPath;
}

// ============================================
// 发送下一帧 (250Hz)
// ============================================
// ============================================
// Online beam-depth fit for the real-time stream.
// Each frame is fitted with the frames received so far (causal), so the
// 250 Hz stream is never blocked by a full-dataset pass.
// ============================================
void Scan::computeBeamDepthOnline()
{
    const int maxBeam = 64;
    const int centerBeam = 24;
    const double spacing = 0.3;
    const double d2r = 0.017453292519943295;

    double A = robot_a*d2r, B = robot_b*d2r, C = robot_c*d2r;
    double ca=cos(A), sa=sin(A), cb=cos(B), sb=sin(B), cc=cos(C), sc=sin(C);
    double R00=ca*cb, R01=ca*sb*sc-sa*cc, R02=ca*sb*cc+sa*sc;
    double R10=sa*cb, R11=sa*sb*sc+ca*cc, R12=sa*sb*cc-ca*sc;
    double R20=-sb,   R21=cb*sc,          R22=cb*cc;

    double tx = robot_x, ty = robot_y, tz = robot_z;
    double siV = si;
    double px0 = tx + R02 * (-siV * 0.5);
    double py0 = ty + R12 * (-siV * 0.5);
    double pz0 = tz + R22 * (-siV * 0.5);

    int idx = (int)m_fitCx.size();
    m_fitCx.push_back((float)px0);
    m_fitCy.push_back((float)py0);
    m_fitCz.push_back((float)pz0);
    m_fitSi.push_back((float)siV);

    int gx = (int)floor(px0 / 12.0);
    int gy = (int)floor(py0 / 12.0);
    long long key = ((long long)(gx + 100000) << 32) | (unsigned int)(gy + 100000);
    m_fitGrid[key].push_back(idx);

    std::array<float, 64> depths;
    for (int j = 0; j < maxBeam; j++) depths[j] = (float)(-siV * 0.5);

    std::vector<double> xs, ys, zs, ws;
    xs.reserve(700); ys.reserve(700); zs.reserve(700); ws.reserve(700);
    double yMin = 1e300, yMax = -1e300;
    for (int ddx = -2; ddx <= 2; ddx++) {
        for (int ddy = -3; ddy <= 3; ddy++) {
            long long k2 = ((long long)(gx + ddx + 100000) << 32) | (unsigned int)(gy + ddy + 100000);
            auto it = m_fitGrid.find(k2);
            if (it == m_fitGrid.end()) continue;
            for (int id2 : it->second) {
                double wx = m_fitCx[id2] - tx;
                double wy = m_fitCy[id2] - ty;
                double wz = m_fitCz[id2] - tz;
                double xl = wx*R00 + wy*R10 + wz*R20;
                double yl = wx*R01 + wy*R11 + wz*R21;
                double zl = wx*R02 + wy*R12 + wz*R22;
                if (fabs(xl) > 12.0 || fabs(yl) > 30.0) continue;
                xs.push_back(xl); ys.push_back(yl); zs.push_back(zl);
                ws.push_back(exp(-(xl*xl + yl*yl) / 100.0));
                if (yl < yMin) yMin = yl;
                if (yl > yMax) yMax = yl;
            }
        }
    }

    int n = (int)xs.size();
    bool fitted = false;
    if (n >= 30 && (yMax - yMin) >= 8.0) {
        double ATA[6][6] = {{0}};
        double ATb[6] = {0};
        for (int s = 0; s < n; s++) {
            double x = xs[s], y = ys[s], z = zs[s], w = ws[s];
            double h = z + siV * 0.5;
            double f[6] = {w, x*w, y*w, x*y*w, x*x*w, y*y*w};
            double b = h * w;
            for (int a = 0; a < 6; a++) {
                ATb[a] += f[a] * b;
                for (int c2 = 0; c2 < 6; c2++) ATA[a][c2] += f[a] * f[c2];
            }
        }
        for (int a = 0; a < 6; a++) ATA[a][a] += 1e-9;

        double M[6][7];
        for (int a = 0; a < 6; a++) {
            for (int c2 = 0; c2 < 6; c2++) M[a][c2] = ATA[a][c2];
            M[a][6] = ATb[a];
        }
        bool ok = true;
        for (int col = 0; col < 6; col++) {
            int piv = col;
            for (int r = col + 1; r < 6; r++)
                if (fabs(M[r][col]) > fabs(M[piv][col])) piv = r;
            if (fabs(M[piv][col]) < 1e-30) { ok = false; break; }
            if (piv != col)
                for (int c2 = col; c2 < 7; c2++) std::swap(M[piv][c2], M[col][c2]);
            for (int r = 0; r < 6; r++) {
                if (r == col) continue;
                double factor = M[r][col] / M[col][col];
                for (int c2 = col; c2 < 7; c2++) M[r][c2] -= factor * M[col][c2];
            }
        }
        if (ok) {
            double coef[6];
            for (int r = 0; r < 6; r++) coef[r] = M[r][6] / M[r][r];
            double symv = (yMax < -yMin) ? yMax : -yMin;
            double wsym = symv / 10.0;
            if (wsym < 0.0) wsym = 0.0;
            if (wsym > 1.0) wsym = 1.0;
            for (int j = 0; j < maxBeam; j++) {
                double y = (j - centerBeam) * spacing;
                if (y < yMin) y = yMin;
                if (y > yMax) y = yMax;
                double hc = coef[0] + wsym * (coef[2] * y + coef[5] * y * y);
                if (hc > 5.0) hc = 5.0;
                if (hc < -5.0) hc = -5.0;
                depths[j] = (float)(-siV * 0.5 + hc);
            }
            m_lastValidFit = idx;
            m_lastValidSi = (float)siV;
            fitted = true;
        }
    }

    if (!fitted && m_lastValidFit >= 0) {
        int dist = idx - m_lastValidFit;
        double alpha = 1.0 - (double)dist / 100.0;
        if (alpha > 0.0) {
            const std::array<float, 64>& best = m_beamDepths[m_lastValidFit];
            for (int j = 0; j < maxBeam; j++) {
                double hc = (double)best[j] + m_lastValidSi * 0.5;
                if (hc > 5.0) hc = 5.0;
                if (hc < -5.0) hc = -5.0;
                depths[j] = (float)(-siV * 0.5 + alpha * hc);
            }
        }
    }

    m_beamDepths.push_back(depths);
    m_beamDepthsReady = true;
}

void Scan::sendNextFrame()
{
    // 方案2：均匀网格点云播放
    if (m_gridReady) {
        if (m_gridIndex >= (int)m_gridCloud.size()) {
            m_isRunning = false;
            m_start = false;
            closeCsvSave();
            emit finished();
            qDebug() << "Grid playback finished";
            return;
        }
        const GridFrame& g = m_gridCloud[m_gridIndex];
        ScanFrame frame;
        frame.ipoc = (quint32)(m_gridIndex + 1);
        frame.si = g.si;
        frame.beam = g.valid;
        for (int i = 0; i < 64; i++) {
            frame.amp[i] = (i < g.valid) ? g.amp[i] : 0.0;
            frame.tof[i] = (i < g.valid) ? g.tof[i] : 0.0;
        }
        memcpy(frame.pose, g.pose, sizeof(double)*6);
        frame.worldXYZ = &g.pts[0].x;
        frame.worldCount = g.valid;
        frame.hasWorld = true;
        m_gridIndex++;
        emit newFrameAvailable(frame);
        return;
    }

    // 数据源：CSV 回放 或 实时全局量（scandata）
    // 与实时写线程（udpserver/ads_poller/viewmodel）互斥：离线加载/回放期间
    // 防止实时线程穿插改写 pose/si/amp 等全局量，导致网格单元错位、大量去重
    // 合并成“缺帧”。
    QMutexLocker locker(&g_scanDataMutex);
    if (m_dataStream.is_open()) {
        // CSV 回放：读下一行
        std::string line;
        std::vector<std::string> cells;
        bool gotFrame = false;
        while (std::getline(m_dataStream, line)) {
            cells = parseCSVLine(line);
            if (!cells.empty() && cells.size() > 10) { gotFrame = true; break; }
        }
        if (!gotFrame) {
            if (m_onlineGridEnabled) {
                if (!m_win.empty() || !m_rowBuf.empty() || m_onlinePacked > 0) {
                    flushOnlineTail();
                }
            }
            m_timer->stop();
            m_isRunning = false;
            m_start = false;
            m_paused = false;
            closeCsvSave();
            if (!m_offlineBuild) emit finished();
            qDebug() << "Playback finished";
            return;
        }
        updateGlobalVariables(cells);
    }
    // 实时模式：全局量由 udpserver（机器人 250Hz/IPOC）和 ViewModel（超声 180Hz）
    // 实时写入。以机器人 IPOC 为硬件时间戳：IPOC 未变化就跳过本 tick。
    if (!m_dataStream.is_open()) {
        quint64 nowMs = (quint64)QDateTime::currentMSecsSinceEpoch();
        // 软暂停：等待机器人 IPOC 停稳后再真正停止。停稳之前照常处理
        // 每一帧（滑行段也画点），保证暂停点附近不缺帧。
        if (m_pausePending) {
            if (robot_ipoc != m_pauseLastIpoc) {
                m_pauseLastIpoc = robot_ipoc;
                m_pauseStableValid = false;
            } else if (!m_pauseStableValid) {
                m_pauseStableTimer.restart();
                m_pauseStableValid = true;
            }
            if (m_pauseStableValid && m_pauseStableTimer.elapsed() >= 800) {
                m_pausePending = false;
                m_pauseStableValid = false;
                m_timer->stop();
                m_isRunning = false;
                m_paused = true;
                m_start = false;
                qDebug() << "Scan paused (IPOC settled)";
                return;
            }
            if (m_pauseStableValid && m_pauseStableTimer.elapsed() > 5000) {
                // 上限保护：PLC 一直没停稳时强制暂停，避免无限等待
                m_pausePending = false;
                m_pauseStableValid = false;
                m_timer->stop();
                m_isRunning = false;
                m_paused = true;
                m_start = false;
                qDebug() << "Scan paused (forced after 5s settle timeout)";
                return;
            }
        }
        if (robot_ipoc == m_lastIpoc) {
            // 停滞检测：实时扫描中 IPOC 长时间不变化，说明机器人数据被阻塞/卡死
            if (m_lastIpocChangeMs != 0 && nowMs - m_lastIpocChangeMs > 500) {
                static quint64 lastStallWarnMs = 0;
                if (nowMs - lastStallWarnMs >= 1000) {
                    lastStallWarnMs = nowMs;
                    qDebug() << "[Scan] IPOC停滞警告: 已" << (nowMs - m_lastIpocChangeMs)
                             << "ms 未变化, robot_ipoc=" << robot_ipoc;
                }
            }
            return;
        }
        // IPOC 增量驱动：每一帧机器人（250Hz）都处理，超声读取最近一次
        // 样本（zero-order hold），保证 250 个 IPOC 帧每个都有点。
        quint32 ipocDelta = robot_ipoc - m_lastIpoc;   // 无符号差，处理回绕
        if (ipocDelta > 4) {
            m_lostFrames += ipocDelta / 4 - 1;
            static quint64 lastMissWarnMs = 0;
            if (nowMs - lastMissWarnMs >= 1000) {
                lastMissWarnMs = nowMs;
                qDebug() << "[Scan] 实时缺帧: 上次IPOC" << m_lastIpoc
                         << " 当前" << robot_ipoc
                         << " 跳过" << (ipocDelta / 4 - 1) << "帧"
                         << " 累计" << m_lostFrames << "帧";
            }
        }
        m_lastIpoc = robot_ipoc;
        m_lastIpocChangeMs = nowMs;
    }
    computeBeamDepthOnline();
    m_currentIndex++;
    // 每处理一帧写一行数据（实时模式落盘；回放模式未打开文件时为空操作）
    writeCsvRow();

    // 发送帧数据
    if (m_onlineGridEnabled) {
        const double d2r = 0.017453292519943295;
        double A = robot_a*d2r, B = robot_b*d2r, C = robot_c*d2r;
        double ca=cos(A), sa=sin(A), cb=cos(B), sb=sin(B), cc=cos(C), sc=sin(C);
        double R00=ca*cb, R01=ca*sb*sc-sa*cc, R02=ca*sb*cc+sa*sc;
        double R10=sa*cb, R11=sa*sb*sc+ca*cc, R12=sa*sb*cc-ca*sc;
        double R20=-sb,   R21=cb*sc,          R22=cb*cc;
        if (m_latCount < 200) {
            m_ugx += R01; m_ugy += R11; m_ugz += R21;
            m_latCount++;
        }
        double ugx=m_ugx, ugy=m_ugy, ugz=m_ugz;
        double un=sqrt(ugx*ugx+ugy*ugy+ugz*ugz); if (un<1e-12) un=1.0;
        ugx/=un; ugy/=un; ugz/=un;
        double vx=-ugy, vy=ugx;
        double vn=sqrt(vx*vx+vy*vy); if (vn<1e-12) vn=1.0;
        vx/=vn; vy/=vn;
        double ux=ugx, uy=ugy;
        double nun=sqrt(ux*ux+uy*uy); if (nun<1e-12) nun=1.0;
        ux/=nun; uy/=nun;
        double vx2=vx, vy2=vy;
        double nvn=sqrt(vx2*vx2+vy2*vy2); if (nvn<1e-12) nvn=1.0;
        vx2/=nvn; vy2/=nvn;

        const std::array<float,64>& bz = m_beamDepths[m_currentIndex-1];
        FrameRecord rec;
        for (int e = 0; e < 49; e++) {
            double y = (e - 24) * 0.3;
            double z = bz[e];
            rec.xyz[e*3+0] = (float)(robot_x + R01*y + R02*z);
            rec.xyz[e*3+1] = (float)(robot_y + R11*y + R12*z);
            rec.xyz[e*3+2] = (float)(robot_z + R21*y + R22*z);
            rec.amp[e] = (float)amp[e];
            rec.tof[e] = (float)tof[e];
        }

        // The grid is built frame by frame: every incoming frame's 49
        // elements become 49 nearest-cell candidates in Scan::processFrameOnline().

        // frame centre raw + smoothed
        double uc0 = rec.xyz[24*3+0]*ugx + rec.xyz[24*3+1]*ugy + rec.xyz[24*3+2]*ugz;
        double sc0 = rec.xyz[24*3+0]*vx + rec.xyz[24*3+1]*vy;
        double zc0 = rec.xyz[24*3+2];
        m_uHist.push_back(uc0);
        m_sHist.push_back(sc0);
        m_zHist.push_back(zc0);
        const int histN = 21;
        while ((int)m_uHist.size() > histN) m_uHist.pop_front();
        while ((int)m_sHist.size() > histN) m_sHist.pop_front();
        while ((int)m_zHist.size() > histN) m_zHist.pop_front();
        double ucS = 0.0, scS = 0.0, zcS = 0.0;
        for (double h : m_uHist) ucS += h;
        for (double h : m_sHist) scS += h;
        for (double h : m_zHist) zcS += h;
        ucS /= (int)m_uHist.size();
        scS /= (int)m_sHist.size();
        zcS /= (int)m_zHist.size();

        // Causal 250 Hz pass detection: no data is read ahead. The first
        // band's reference centre is frozen after a short warm-up and every
        // later band is placed on the regular 14.4 mm grid from it.
        double dS = 0.0;
        if (m_lastScSValid) dS = scS - m_lastScS;
        m_lastScS = scS;
        m_lastScSValid = true;
        int dir = (dS > 0.05) ? 1 : (dS < -0.05) ? -1 : 0;
        // Hysteresis: a direction change only commits once it has
        // persisted for kPassHystFrames, so turn wiggles cannot
        // double-increment the pass index (which would leave gaps
        // between parallel path lines in the grid).
        const int kPassHystFrames = 25;
        if (dir != 0 && dir == m_candDir) m_candDirCnt++;
        else { m_candDir = dir; m_candDirCnt = (dir != 0) ? 1 : 0; }
        // 防抖窗口从 60 帧收到 30 帧（120ms）：掉头提交更快，
        // 避免真实换向时机器人已退回一段距离、同一带内出现双向行号。
        if (dir != 0 && m_candDirCnt >= kPassHystFrames && m_passDir != 0 && dir != m_passDir
            && m_currentIndex - m_lastFlipFrame >= 30) {
            if (!m_pass0RefFrozen && m_passUcnt > 0) {
                m_pass0Ref = m_passUsum / m_passUcnt;
                m_pass0RefFrozen = true;
            }
            finalizeAllRows();
            // Emit any partially-filled pack with the OLD pass index first,
            // so a pack never mixes cells from two bands.
            flushOnlinePack();
            // Reset the cell dedupe so every band emits its full 49-column
            // footprint (the shared edge column is re-covered by the new
            // band instead of being swallowed by the previous band's cells).
            m_onlineCells.clear();
            m_passIndex++;
            m_passDir = dir;
            m_passFrames = 0;
            m_candDirCnt = 0;
            m_lastFlipFrame = m_currentIndex;
        }
        if (m_passDir == 0) m_passDir = dir;
        m_passFrames++;

        PassFrameMeta meta;
        meta.uc0 = uc0; meta.ucS = ucS;
        meta.sc0 = sc0; meta.scS = scS;
        meta.zc0 = zc0; meta.zcS = zcS;
        meta.turn = (dir == 0 || m_passFrames <= 100);
        processFrameOnline(rec, meta, dir);
    } else {
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
        if (m_beamDepthsReady && m_currentIndex > 0 && m_currentIndex <= (int)m_beamDepths.size()) {
            const std::array<float, 64>& bz2 = m_beamDepths[m_currentIndex - 1];
            for (int j2 = 0; j2 < 64; j2++) frame.beamZ[j2] = (double)bz2[j2];
        } else {
            for (int j2 = 0; j2 < 64; j2++) frame.beamZ[j2] = -si * 0.5;
        }
        emit newFrameAvailable(frame);
    }

    // // ✅ 打印实时数据
    // qDebug() << "Frame:" << frame.ipoc
    //          << "SI:" << frame.si
    //          << "Beam:" << frame.beam
    //          << "Pose:" << frame.pose[0] << frame.pose[1] << frame.pose[2]
    //          << frame.pose[3] << frame.pose[4] << frame.pose[5]
    //          << "AMP[0]:" << frame.amp[0]
    //          << "TOF[0]:" << frame.tof[0];

    // 在 Scan::sendNextFrame() 里

}
void Scan::processFrameOnline(const FrameRecord& rec, const PassFrameMeta& meta, int dir)
{
    // Skip the first ~30 startup frames: the u/s/z smoothing histograms and
    // the band reference are still settling, so their rows would be placed
    // at a slightly shifted position and appear as a detached line next to
    // the first path line.
    if (m_currentIndex < 30) return;

    double un = sqrt(m_ugx*m_ugx + m_ugy*m_ugy + m_ugz*m_ugz);
    double ugx = m_ugx, ugy = m_ugy, ugz = m_ugz;
    if (un < 1e-12) un = 1.0;
    ugx /= un; ugy /= un; ugz /= un;
    double vx = -ugy, vy = ugx;
    double vn = sqrt(vx*vx + vy*vy); if (vn < 1e-12) vn = 1.0;
    vx /= vn; vy /= vn;
    double ux = ugx, uy = ugy;
    double nun = sqrt(ux*ux + uy*uy); if (nun < 1e-12) nun = 1.0;
    ux /= nun; uy /= nun;
    double vx2 = vx, vy2 = vy;
    double nvn = sqrt(vx2*vx2 + vy2*vy2); if (nvn < 1e-12) nvn = 1.0;
    vx2 /= nvn; vy2 /= nvn;

    // AMP/TOF frame-offset pairing: colours of frame f come from frame f+d.
    // With d<0 the needed frame is already in the history ring.
    m_colorHist.push_back(rec);
    int ringN = 1 - m_usDelayFrames;
    if (ringN < 1) ringN = 1;
    while ((int)m_colorHist.size() > ringN) m_colorHist.pop_front();
    const FrameRecord& col = m_colorHist.front();
    // Lateral reference for this frame, decided only from data already
    // received (no pre-scan). The first band accumulates a running centre;
    // once it is frozen (150 straight frames or at the first direction
    // change), band N sits at ref + sign * N * 14.4 mm.
    if (m_passIndex == 0 && !m_pass0RefFrozen) {
        m_passUsum += meta.ucS;
        m_passUcnt++;
        // Freeze the first band's reference early (frame 30): the u of the
        // startup frames is already stable, and a late freeze makes band 0's
        // rows slide in k and appear as a detached line at the scan start.
        if (m_passFrames == 30) {
            m_pass0Ref = m_passUsum / m_passUcnt;
        }
        if (m_passFrames >= 30) m_pass0RefFrozen = true;
    }
    double uRef;
    if (!m_pass0RefFrozen) {
        uRef = meta.ucS;
    } else if (m_passIndex == 0) {
        uRef = m_pass0Ref;
    } else {
        if (m_passStepSign == 0) {
            double du = meta.uc0 - m_pass0Ref;
            if (fabs(du) > 2.0)
                m_passStepSign = (du < 0) ? -1 : 1;
        }
        if (m_passStepSign != 0) {
            uRef = m_pass0Ref + m_passStepSign * (double)m_passIndex * 14.4;
        } else {
            uRef = meta.ucS;
        }
    }

    // Nearest-cell candidates of this frame (49 elements -> 49 cells).
    std::unordered_map<long long, CandCell> cmap;
    cmap.reserve(49);
    // Snap the whole frame to a single s-row (the smoothed centre) so every
    // frame produces one complete row instead of several partial rows.
    const long long j = (long long)floor(meta.scS / 0.7);
    // The k columns come from the element's position along the array
    // ((e-24)*0.3 mm), not from its projection onto the frozen reference
    // axis. The tool rotates along the path, so the projected span shrinks
    // to 46..47 columns; using the along-array distance keeps the full
    // 49 columns (14.4 mm) so adjacent path strips tile without gaps.
    const long long kc = (long long)floor(uRef / 0.3);
    for (int e = 0; e < 49; e++) {
        if (!beamValid[e]) continue;  // 板外波束不进入在线网格
        double px = rec.xyz[e*3+0];
        double py = rec.xyz[e*3+1];
        double pz = rec.xyz[e*3+2];
        double sg = px*vx + py*vy;
        double us = uRef + (e - 24) * 0.3;
        double ss = (sg - meta.sc0) + meta.scS;
        double zs = (pz - meta.zc0) + meta.zcS;
        long long k = kc + (e - 24);
        double du = us - ((double)k + 0.5) * 0.3;
        double ds = ss - ((double)j + 0.5) * 0.7;
        double d2 = du*du + ds*ds;
        long long key = (k << 32) | (unsigned int)(j & 0xFFFFFFFFLL);
        CandCell cc;
        cc.k = k; cc.j = j;
        cc.z = (float)zs; cc.a = col.amp[e]; cc.t = col.tof[e];
        cc.d2 = d2;
        cc.e = e;
        auto it = cmap.find(key);
        if (it == cmap.end() || d2 < it->second.d2) cmap[key] = cc;
    }

    // Frame-by-frame grid build with a short lookahead: a cell is emitted
    // once, ~m_winMax frames after its first candidate, using the candidate
    // closest to the cell centre among all frames in the window.
    m_win.push_back(std::move(cmap));
    while ((int)m_win.size() > m_winMax) {
        std::unordered_map<long long, CandCell> oldest = std::move(m_win.front());
        m_win.pop_front();
        for (auto& kv : oldest) {
            CandCell best = kv.second;
            for (auto& wm : m_win) {
                auto it = wm.find(kv.first);
                if (it != wm.end() && it->second.d2 < best.d2) best = it->second;
            }
            if (best.e >= 0 && !beamValid[best.e]) continue;  // 板外波束丢弃，清除窗口拖尾
            insertRowCell(best.k, best.j, best.z, best.a, best.t, best.e, ux, uy, vx2, vy2);
        }
    }

    if (dir != 0) m_bandDirFwd = (dir == 1);
    // 转弯停滞处理：转弯帧（dir==0）时立即发射未完成的行，避免积压到翻转时爆发。
    if (dir == 0) {
        for (auto it = m_rowBuf.begin(); it != m_rowBuf.end(); ++it)
            if (!it->second.done) {
                finalizeFilteredRow(it->first);
                it->second.done = true;
            }
    }
}

void Scan::insertRowCell(long long k, long long j, float z, float a, float t, int e,
                         double ux, double uy, double vx2, double vy2)
{
    if (e >= 0 && !beamValid[e]) return;  // 延迟进入行缓存的板外波束直接丢弃
    if (!m_rowBufActive) {
        m_rowBufActive = true;
        m_bandDirFwd = true;
        m_nextFinalizeJ = j;
    }
    auto itr0 = m_rowBuf.find(j);
    if (itr0 != m_rowBuf.end() && itr0->second.done) {
        // ???????????????????????????????
        emitGridCell(k, j, z, a, t, ux, uy, vx2, vy2);
        return;
    }
    m_rowBuf[j].cells[k] = RowCell{z, a, t, e};

    if (m_bandDirFwd) {
        while (m_nextFinalizeJ <= j - m_filterWin) {
            auto it = m_rowBuf.find(m_nextFinalizeJ);
            if (it != m_rowBuf.end() && !it->second.done) {
                finalizeFilteredRow(m_nextFinalizeJ);
                it->second.done = true;
            }
            m_nextFinalizeJ++;
        }
    } else {
        while (m_nextFinalizeJ >= j + m_filterWin) {
            auto it = m_rowBuf.find(m_nextFinalizeJ);
            if (it != m_rowBuf.end() && !it->second.done) {
                finalizeFilteredRow(m_nextFinalizeJ);
                it->second.done = true;
            }
            m_nextFinalizeJ--;
        }
    }
}

void Scan::finalizeFilteredRow(long long j)
{
    auto itj = m_rowBuf.find(j);
    if (itj == m_rowBuf.end()) return;
    double un = sqrt(m_ugx*m_ugx + m_ugy*m_ugy + m_ugz*m_ugz);
    double ugx = m_ugx, ugy = m_ugy, ugz = m_ugz;
    if (un < 1e-12) un = 1.0;
    ugx /= un; ugy /= un; ugz /= un;
    double vx = -ugy, vy = ugx;
    double vn = sqrt(vx*vx + vy*vy); if (vn < 1e-12) vn = 1.0;
    vx /= vn; vy /= vn;
    double ux = ugx, uy = ugy;
    double nun = sqrt(ux*ux + uy*uy); if (nun < 1e-12) nun = 1.0;
    ux /= nun; uy /= nun;
    double vx2 = vx, vy2 = vy;
    double nvn = sqrt(vx2*vx2 + vy2*vy2); if (nvn < 1e-12) nvn = 1.0;
    vx2 /= nvn; vy2 /= nvn;

    const RowBuf& rb = itj->second;
    for (auto& kv : rb.cells) {
        long long k = kv.first;
        const RowCell& c = kv.second;
        if (c.e >= 0 && !beamValid[c.e]) continue;  // 板外波束不发射
        // 2D ????????? k??? j ? ?W???? AMP/TOF ???z ???
        float a = c.a, t = c.t;
        // 小噪点支持度抑制：3x3 原始值中位数作背景，
        // 孤立的1~2格噪点无同向邻居支持，压回背景；≥1mm 缺陷有足够支持被保留。
        std::array<float, 9> na, nt;
        int nn = 0;
        for (long long dj = -1; dj <= 1; dj++) {
            auto itr = m_rowBuf.find(j + dj);
            if (itr == m_rowBuf.end()) continue;
            for (long long dk = -1; dk <= 1; dk++) {
                auto itc = itr->second.cells.find(k + dk);
                if (itc == itr->second.cells.end()) continue;
                if (nn < 9) { na[nn] = itc->second.a; nt[nn] = itc->second.t; nn++; }
            }
        }
        float bgA = c.a, bgT = c.t;
        if (nn >= 3) {
            std::nth_element(na.begin(), na.begin() + nn / 2, na.begin() + nn);
            std::nth_element(nt.begin(), nt.begin() + nn / 2, nt.begin() + nn);
            bgA = na[nn / 2];
            bgT = nt[nn / 2];
        }
        float devA = c.a - bgA;
        float devT = c.t - bgT;
        bool candA = (fabsf(devA) > kAmpNoiseDev);
        bool candT = (fabsf(devT) > kTofNoiseDev);
        if (candA || candT) {
            int supA = 0, supT = 0;
            for (long long dj = -1; dj <= 1; dj++) {
                auto itr = m_rowBuf.find(j + dj);
                if (itr == m_rowBuf.end()) continue;
                for (long long dk = -1; dk <= 1; dk++) {
                    if (dj == 0 && dk == 0) continue;
                    auto itc = itr->second.cells.find(k + dk);
                    if (itc == itr->second.cells.end()) continue;
                    float dA = itc->second.a - bgA;
                    float dT = itc->second.t - bgT;
                    if (candA && dA * devA > 0.f && fabsf(dA) > 0.5f * fabsf(devA)) supA++;
                    if (candT && dT * devT > 0.f && fabsf(dT) > 0.5f * fabsf(devT)) supT++;
                }
            }
            if (candA && supA < kMinSupport) { a = bgA; m_noiseSuppressed++; }
            if (candT && supT < kMinSupport) { t = bgT; m_noiseSuppressed++; }
        }
        emitGridCell(k, j, c.z, a, t, ux, uy, vx2, vy2);
    }
}

void Scan::finalizeAllRows()
{
    if (!m_rowBufActive) return;
    if (m_bandDirFwd) {
        for (auto it = m_rowBuf.begin(); it != m_rowBuf.end(); ++it)
            if (!it->second.done) {
                finalizeFilteredRow(it->first);
                it->second.done = true;
            }
    } else {
        for (auto it = m_rowBuf.rbegin(); it != m_rowBuf.rend(); ++it)
            if (!it->second.done) {
                finalizeFilteredRow(it->first);
                it->second.done = true;
            }
    }
    m_rowBuf.clear();
    m_rowBufActive = false;
    m_bandDirFwd = true;
    m_nextFinalizeJ = 0;
}

void Scan::emitGridCell(long long k, long long j, float z, float a, float t,
                        double ux, double uy, double vx2, double vy2)
{

    long long key = (k << 32) | (unsigned int)(j & 0xFFFFFFFFLL);
    if (m_onlineCells.find(key) != m_onlineCells.end()) {
        return;
    }
    OnlineCell c; c.z = z; c.amp = a; c.tof = t;
    m_onlineCells.emplace(key, c);
    if (m_onlinePacked == 49) flushOnlinePack();
    double uc = k * 0.3;
    double sc = j * 0.7;
    m_onlineGf.pts[m_onlinePacked].x = (float)(uc*ux + sc*vx2);
    m_onlineGf.pts[m_onlinePacked].y = (float)(uc*uy + sc*vy2);
    m_onlineGf.pts[m_onlinePacked].z = z;
    m_onlineGf.amp[m_onlinePacked] = a;
    m_onlineGf.tof[m_onlinePacked] = t;
    m_onlineGf.gk[m_onlinePacked] = (int)k;
    m_onlineGf.gj[m_onlinePacked] = (int)j;
    m_onlinePacked++;
}

void Scan::flushOnlinePack()
{
    if (m_onlinePacked <= 0) return;
    m_onlineGf.valid = m_onlinePacked;
    // 记录该包对应的 TCP 位姿 / SI / 带序号（离线加载时用于重建 ScanFrame）
    m_onlineGf.si = (float)si;
    m_onlineGf.passIndex = m_passIndex;
    m_onlineGf.pose[0] = robot_x - m_lastLY;
    m_onlineGf.pose[1] = robot_y + m_lastLX;
    m_onlineGf.pose[2] = robot_z;
    m_onlineGf.pose[3] = robot_a;
    m_onlineGf.pose[4] = robot_b;
    m_onlineGf.pose[5] = robot_c;
    m_onlineOut.push_back(m_onlineGf);
    const GridFrame& g = m_onlineOut.back();
    ScanFrame f;
    f.ipoc = robot_ipoc;
    f.si = g.si;
    // TCP pose (undo the LX/LY transducer offset so the file matches
    // the imported CSV format and can be re-imported).
    memcpy(f.pose, g.pose, sizeof(double) * 6);
    f.lx = m_lastLX;
    f.ly = m_lastLY;
    f.beam = g.valid;
    for (int i = 0; i < 64; i++) {
        f.amp[i] = (i < g.valid) ? g.amp[i] : 0.0;
        f.tof[i] = (i < g.valid) ? g.tof[i] : 0.0;
    }
    f.worldXYZ = &g.pts[0].x;
    f.worldCount = g.valid;
    f.hasWorld = true;
    f.gridK = g.gk.data();
    f.gridJ = g.gj.data();
    f.hasGrid = true;
    f.passIndex = g.passIndex;
    m_onlineIndex++;
    if (!m_suppressEmit) emit newFrameAvailable(f);
    m_onlineGf = GridFrame();
    m_onlinePacked = 0;
}

void Scan::flushOnlineTail()
{
    if (!m_onlineGridEnabled) return;
    double un = sqrt(m_ugx*m_ugx + m_ugy*m_ugy + m_ugz*m_ugz);
    double ugx = m_ugx, ugy = m_ugy, ugz = m_ugz;
    if (un < 1e-12) un = 1.0;
    ugx /= un; ugy /= un; ugz /= un;
    double vx = -ugy, vy = ugx;
    double vn = sqrt(vx*vx + vy*vy); if (vn < 1e-12) vn = 1.0;
    vx /= vn; vy /= vn;
    double ux = ugx, uy = ugy;
    double nun = sqrt(ux*ux + uy*uy); if (nun < 1e-12) nun = 1.0;
    ux /= nun; uy /= nun;
    double vx2 = vx, vy2 = vy;
    double nvn = sqrt(vx2*vx2 + vy2*vy2); if (nvn < 1e-12) nvn = 1.0;
    vx2 /= nvn; vy2 /= nvn;

    while (!m_win.empty()) {
        std::unordered_map<long long, CandCell> oldest = std::move(m_win.front());
        m_win.pop_front();
        for (auto& kv : oldest) {
            CandCell best = kv.second;
            for (auto& wm : m_win) {
                auto it = wm.find(kv.first);
                if (it != wm.end() && it->second.d2 < best.d2) best = it->second;
            }
            if (best.e >= 0 && !beamValid[best.e]) continue;
            insertRowCell(best.k, best.j, best.z, best.a, best.t, best.e, ux, uy, vx2, vy2);
        }
    }
    finalizeAllRows();
    flushOnlinePack();
}



// ============================================
// 开始数据采集
// ============================================
void Scan::start()
{
    if (m_pausePending) {
        // 软暂停等待 IPOC 停稳期间又要求开始：取消软暂停，继续运行
        // （此时 m_isRunning 仍为 true，必须先处理再走下面的 resume 逻辑）
        m_pausePending = false;
        m_pauseStableValid = false;
        m_isRunning = true;
        m_paused = false;
        m_start = true;
        if (m_dataStream.is_open())
            m_timer->start(4);   // CSV 回放：250Hz 定时器
        else
            m_timer->start(1);   // 实时模式：1ms 轮询 IPOC
        qDebug() << "Scan resume requested during settle (pause cancelled)";
        return;
    }

    if (m_isRunning) {
        return;
    }

    if (m_paused) {
        // 暂停后继续：流位置和网格缓冲保留，直接重启定时器
        m_isRunning = true;
        m_paused = false;
        m_start = true;
        if (m_dataStream.is_open())
            m_timer->start(4);   // CSV 回放：250Hz 定时器
        else
            m_timer->start(1);   // 实时模式：1ms 轮询 IPOC，保证不漏硬件时间戳
        qDebug() << "Scan resumed at frame" << m_currentIndex;
        return;
    }

    // Fresh start: reset all online-grid / pass-detection state, otherwise a
    // second scan inherits the previous scan's band index, dedupe table and
    // reference axis and the surface becomes corrupted.
    m_onlineCells.clear();
    m_onlineOut.clear();
    m_loadedFrames.clear();
    m_onlineGf = GridFrame();
    m_onlinePacked = 0;
    m_onlineIndex = 0;
    m_uHist.clear(); m_sHist.clear(); m_zHist.clear();
    m_frameRing.clear();
    m_latCount = 0;
    m_ugx = m_ugy = m_ugz = 0;
    m_passDir = 0;
    m_passFrames = 0;
    m_passUsum = 0;
    m_passUcnt = 0;
    m_prevPassCenter = 0;
    m_prevPassCenterValid = false;
    m_lastScS = 0;
    m_lastScSValid = false;
    m_curShift = 0;
    m_passStepSign = 0;
    m_pass0RefFrozen = false;
    m_pass0Ref = 0;
    m_passIndex = 0;
    m_candDir = 0;
    m_candDirCnt = 0;
    m_lastFlipFrame = -100000;
    m_paused = false;
    m_pausePending = false;
    m_pauseStableValid = false;
    m_rowBuf.clear();
    m_rowBufActive = false;
    m_bandDirFwd = true;
    m_nextFinalizeJ = 0;
    m_noiseSuppressed = 0;
    m_win.clear();
    m_colorHist.clear();
    // 波束深度/曲面拟合状态也属上一轮扫描：不清空的话第二轮扫描会沿用
    // 上一轮的深度拟合与空间哈希（m_beamDepths 索引错位、点位 z 错乱）。
    m_beamDepths.clear();
    m_beamDepthsReady = false;
    m_fitCx.clear(); m_fitCy.clear(); m_fitCz.clear(); m_fitSi.clear();
    m_fitGrid.clear();
    m_lastValidFit = -1;
    m_lastValidSi = 0.0f;
    m_gridCloud.clear();
    m_gridReady = false;
    m_gridIndex = 0;

    m_isRunning = true;
    m_currentIndex = 0;
    m_gridIndex = 0;
    m_lastIpoc = 0;
    m_lostFrames = 0;
    m_start = true;

    if (!m_csvPath.empty()) {
        // CSV 回放模式
        robot_ipoc = 0;
        m_dataStream.close();
        m_dataStream.open(m_csvPathW.c_str(), std::ios::in);
        if (m_dataStream.is_open()) {
            std::string header;
            std::getline(m_dataStream, header);   // skip header row
        }
        m_timer->start(4);  // CSV 回放：定时器驱动 250Hz
    } else {
        // 实时模式：无 CSV 文件，数据由 udpserver（机器人 250Hz/IPOC）和
        // ViewModel（超声 180Hz）实时写入 scandata 全局量。
        // 用 1ms 定时器实时轮询 IPOC：机器人 IPOC 每 4ms 变化一次，
        // 1ms 采样间隔保证不会漏掉任何硬件时间戳（检测到变化才处理一帧）。
        m_lastIpoc = robot_ipoc;
        m_timer->start(1);
        qDebug() << "Real-time mode: reading live robot/ultrasound data";
        // 实时扫描数据统一由 Scan 线程落盘（SoundScan 的 CSV 线程已移除）
        if (m_saveCsv) openCsvSave();
    }

    qDebug() << "Data acquisition started at 250Hz";
}

// ============================================
// 停止数据采集
// ============================================
void Scan::stop()
{
    if (m_isRunning) {
        m_pausePending = false;
        m_pauseStableValid = false;
        // Flush the buffered pass and pending grid cells so a manual stop
        // does not drop the last path band / queued points.
        if (m_onlineGridEnabled) {
            if (!m_win.empty() || !m_rowBuf.empty() || m_onlinePacked > 0) {
                flushOnlineTail();
            }
        }
        m_timer->stop();
        closeCsvSave();
        m_isRunning = false;
        m_start = false;
        m_paused = false;
        emit finished();
        qDebug() << "Scan stopped";
    }
}

// ============================================
// 离线加载：用与“开始绘制”完全相同的在线网格管线处理整个 CSV
// （无 250Hz 定时器节流，按最快速度跑完），期间不发 newFrameAvailable，
// 完成后通过 buildFinished 通知；网格帧在 loadedFrames() 中可逐帧喂给
// 同一渲染路径（renderFrame → AddPointCloud worldXYZ），因此加载结果
// 与回放逐帧绘制结果一致。
// 必须在 Scan 线程内调用（MainWindow3 通过 QueuedConnection invoke），
// 且调用前应确保扫描已停止。
// ============================================
void Scan::loadCSVAndBuild(const QString& filename)
{
    m_buildCancel = false;
    m_offlineBuild = true;
    if (!loadCSVFile(filename)) {
        m_offlineBuild = false;
        qWarning() << "loadCSVAndBuild: failed to load" << filename;
        emit buildFinished(false, 0);
        return;
    }

    m_dataStream.close();
    m_dataStream.open(m_csvPathW.c_str(), std::ios::in);
    if (!m_dataStream.is_open()) {
        m_offlineBuild = false;
        qWarning() << "loadCSVAndBuild: cannot open" << filename;
        emit buildFinished(false, 0);
        return;
    }
    std::string header;
    std::getline(m_dataStream, header);   // skip header row

    // 预扫描一次统计总行数（进度显示用）
    int total = 0;
    std::string line;
    std::streampos dataStart = m_dataStream.tellg();
    while (std::getline(m_dataStream, line)) total++;
    m_dataStream.clear();
    m_dataStream.seekg(dataStart);

    // 与 start() 全新开始一致的运行状态（loadCSVFile 已清空网格/带状态）
    m_isRunning = true;
    m_currentIndex = 0;
    m_gridIndex = 0;
    m_lastIpoc = 0;
    m_lostFrames = 0;
    m_start = true;
    robot_ipoc = 0;

    m_suppressEmit = true;
    int done = 0;
    while (m_dataStream.peek() != std::char_traits<char>::eof() && !m_buildCancel.load()) {
        sendNextFrame();
        done++;
        if ((done & 0xFF) == 0)
            emit buildProgress(done, total);
    }
    flushOnlineTail();
    m_suppressEmit = false;

    // 把网格包转成 ScanFrame（worldXYZ 指向 m_onlineOut 内网格数据；
    // 下一次 load/start 前必须消费完毕）
    m_loadedFrames.clear();
    m_loadedFrames.reserve(m_onlineOut.size());
    for (const GridFrame& g : m_onlineOut) {
        ScanFrame f;
        f.ipoc = (quint32)(m_loadedFrames.size() + 1);
        f.si = g.si;
        f.beam = g.valid;
        for (int i = 0; i < 64; i++) {
            f.amp[i] = (i < g.valid) ? g.amp[i] : 0.0;
            f.tof[i] = (i < g.valid) ? g.tof[i] : 0.0;
        }
        memcpy(f.pose, g.pose, sizeof(double) * 6);
        f.worldXYZ = &g.pts[0].x;
        f.worldCount = g.valid;
        f.hasWorld = true;
        f.gridK = g.gk.data();
        f.gridJ = g.gj.data();
        f.hasGrid = true;
        f.passIndex = g.passIndex;
        m_loadedFrames.push_back(f);
    }

    m_dataStream.close();
    m_offlineBuild = false;
    qDebug() << "loadCSVAndBuild: rows" << done << ", grid frames" << m_loadedFrames.size();
    emit buildProgress(total, total);
    emit buildFinished(!m_buildCancel.load(), (int)m_loadedFrames.size());
}

void Scan::cancelBuild()
{
    m_buildCancel = true;
}

// ============================================
// 完全重置扫描状态：停止采集、清除已加载文件/网格/暂停状态，
// 供“重置数据”按钮使用（必须在 Scan 线程内调用，避免与 sendNextFrame 竞争）。
// ============================================
void Scan::resetForNewScan()
{
    m_timer->stop();
    m_isRunning = false;
    m_start = false;
    m_paused = false;
    m_pausePending = false;
    m_pauseStableValid = false;
    m_dataStream.close();
    m_csvPath.clear();
    m_csvPathW.clear();
    m_data.clear();
    m_columnMap.clear();
    m_beamDepths.clear();
    m_beamDepthsReady = false;
    m_fitCx.clear(); m_fitCy.clear(); m_fitCz.clear(); m_fitSi.clear();
    m_fitGrid.clear();
    m_lastValidFit = -1;
    m_lastValidSi = 0.0f;
    m_currentIndex = 0;
    m_totalFrames = 0;
    m_gridIndex = 0;
    m_lastIpoc = 0;
    m_lostFrames = 0;
    m_loadedFrames.clear();
    m_onlineCells.clear();
    m_onlineOut.clear();
    m_onlineGf = GridFrame();
    m_onlinePacked = 0;
    m_onlineIndex = 0;
    m_uHist.clear(); m_sHist.clear(); m_zHist.clear();
    m_frameRing.clear();
    m_latCount = 0;
    m_ugx = m_ugy = m_ugz = 0;
    m_passDir = 0;
    m_passFrames = 0;
    m_passUsum = 0;
    m_passUcnt = 0;
    m_prevPassCenter = 0;
    m_prevPassCenterValid = false;
    m_lastScS = 0;
    m_lastScSValid = false;
    m_curShift = 0;
    m_passStepSign = 0;
    m_pass0RefFrozen = false;
    m_pass0Ref = 0;
    m_passIndex = 0;
    m_candDir = 0;
    m_candDirCnt = 0;
    m_lastFlipFrame = -100000;
    m_rowBuf.clear();
    m_rowBufActive = false;
    m_bandDirFwd = true;
    m_nextFinalizeJ = 0;
    m_noiseSuppressed = 0;
    m_win.clear();
    m_colorHist.clear();
    m_lastLX = 0.0;
    m_lastLY = 0.0;
    m_gridCloud.clear();
    m_gridReady = false;
    m_lastSavedIpoc = 0;
    m_lastSavedIpocValid = false;
    qDebug() << "Scan state reset for new scan";
}

// ============================================
// 暂停数据采集（保留位置，可继续）
// ============================================
void Scan::pause()
{
    if (m_isRunning) {
        // 实时模式：软暂停。暂停命令到达后机器人还要减速滑行一小段，
        // 若立即停表，滑行段的 IPOC 帧会全部丢失，恢复后暂停点附近缺帧。
        // 改为继续轮询并正常处理帧，直到 IPOC 停稳约 800ms 再真正停止。
        if (!m_dataStream.is_open()) {
            m_pausePending = true;
            m_pauseLastIpoc = robot_ipoc;
            m_pauseStableValid = false;
            m_pauseStableTimer.invalidate();
            qDebug() << "Scan pause pending (waiting for IPOC to settle)";
            return;
        }
        // CSV 回放：直接停（流位置保留，恢复时从原位置继续，无滑行问题）
        m_timer->stop();
        m_isRunning = false;
        m_paused = true;
        m_start = false;
        qDebug() << "Scan paused";
    }
}

// ============================================
// 判断是否已加载文件
// ============================================
bool Scan::isFileLoaded() const
{
    return !m_csvPath.empty();
}
