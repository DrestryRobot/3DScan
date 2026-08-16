#include "mainwindow7.h"
#include "ui_mainwindow7.h"
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <vtkCullerCollection.h>

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

// 曲面分块大小：已完成的块不再重传，避免网格越大越卡
static const vtkIdType kChunkMaxCells = 100000;

static const uint32_t color_Amplitude[] = {
        0xffffffff, 0xfffafcfe, 0xfff6fafd, 0xfff2f7fd, 0xffeef5fc, 0xffeaf2fb, 0xffe6f0fb, 0xffe1edfa,
        0xffddebf9, 0xffd9e8f9, 0xffd5e6f8, 0xffd1e3f7, 0xffcde1f7, 0xffc8def6, 0xffc4dcf6, 0xffc0d9f5,
        0xffbcd7f4, 0xffb8d4f4, 0xffb4d2f3, 0xffafd0f2, 0xffabcdf2, 0xffa7cbf1, 0xffa3c8f0, 0xff9fc6f0,
        0xff9bc3ef, 0xff96c1ef, 0xff92beee, 0xff8ebced, 0xff8ab9ed, 0xff86b7ec, 0xff82b4eb, 0xff7db2eb,
        0xff79afea, 0xff75ade9, 0xff71aae9, 0xff6da8e8, 0xff69a6e8, 0xff66a1e5, 0xff639de2, 0xff6099df,
        0xff5d95dc, 0xff5a91da, 0xff588dd7, 0xff5589d4, 0xff5285d1, 0xff4f81cf, 0xff4c7dcc, 0xff4979c9,
        0xff4775c6, 0xff4471c3, 0xff416dc1, 0xff3e69be, 0xff3b65bb, 0xff3861b8, 0xff365db6, 0xff3359b3,
        0xff3055b0, 0xff2d51ad, 0xff2a4daa, 0xff2749a8, 0xff2545a5, 0xff2241a2, 0xff1f3d9f, 0xff1c399d,
        0xff19359a, 0xff163197, 0xff142d94, 0xff112991, 0xff0e258f, 0xff0b218c, 0xff081d89, 0xff051986,
        0xff031584, 0xff041883, 0xff061c83, 0xff082083, 0xff0a2483, 0xff0c2883, 0xff0e2c83, 0xff103082,
        0xff123482, 0xff143882, 0xff153c82, 0xff174082, 0xff194482, 0xff1b4881, 0xff1d4c81, 0xff1f5081,
        0xff215481, 0xff235881, 0xff255c81, 0xff266080, 0xff286480, 0xff2a6880, 0xff2c6c80, 0xff2e7080,
        0xff307480, 0xff32787f, 0xff347c7f, 0xff36807f, 0xff37847f, 0xff39887f, 0xff3b8c7f, 0xff3d907e,
        0xff3f947e, 0xff41987e, 0xff439c7e, 0xff45a07e, 0xff47a47e, 0xff4ca67b, 0xff51a878, 0xff56aa75,
        0xff5bac72, 0xff60ae6f, 0xff65b06c, 0xff6ab269, 0xff6fb467, 0xff74b764, 0xff79b961, 0xff7ebb5e,
        0xff83bd5b, 0xff88bf58, 0xff8dc155, 0xff92c353, 0xff97c550, 0xff9cc74d, 0xffa1ca4a, 0xffa6cc47,
        0xffabce44, 0xffb0d041, 0xffb5d23f, 0xffbad43c, 0xffbfd639, 0xffc4d836, 0xffc9da33, 0xffcedd30,
        0xffd3df2d, 0xffd8e12b, 0xffdde328, 0xffe2e525, 0xffe7e722, 0xffece91f, 0xfff1eb1c, 0xfff6ed19,
        0xfffcf017, 0xfffaec19, 0xfff9e91b, 0xfff8e61d, 0xfff7e31f, 0xfff6df22, 0xfff5dc24, 0xfff4d926,
        0xfff2d628, 0xfff1d32b, 0xfff0cf2d, 0xffefcc2f, 0xffeec931, 0xffedc633, 0xffecc236, 0xffeabf38,
        0xffe9bc3a, 0xffe8b93c, 0xffe7b63f, 0xffe6b241, 0xffe5af43, 0xffe4ac45, 0xffe2a947, 0xffe1a54a,
        0xffe0a24c, 0xffdf9f4e, 0xffde9c50, 0xffdd9953, 0xffdc9555, 0xffda9257, 0xffd98f59, 0xffd88c5b,
        0xffd7885e, 0xffd68560, 0xffd58262, 0xffd47f64, 0xffd37c67, 0xffd27b64, 0xffd27b62, 0xffd27b60,
        0xffd27a5e, 0xffd17a5b, 0xffd17a59, 0xffd17957, 0xffd17955, 0xffd07953, 0xffd07850, 0xffd0784e,
        0xffd0784c, 0xffcf774a, 0xffcf7747, 0xffcf7745, 0xffcf7643, 0xffce7641, 0xffce763f, 0xffce753c,
        0xffce753a, 0xffcd7538, 0xffcd7436, 0xffcd7433, 0xffcd7431, 0xffcc732f, 0xffcc732d, 0xffcc732b,
        0xffcc7228, 0xffcb7226, 0xffcb7224, 0xffcb7122, 0xffcb711f, 0xffca711d, 0xffca701b, 0xffca7019,
        0xffca7017, 0xffc86d17, 0xffc66a17, 0xffc56717, 0xffc36417, 0xffc26217, 0xffc05f18, 0xffbe5c18,
        0xffbd5918, 0xffbb5718, 0xffba5418, 0xffb85118, 0xffb74e19, 0xffb54b19, 0xffb34919, 0xffb24619,
        0xffb04319, 0xffaf4019, 0xffad3e1a, 0xffab3b1a, 0xffaa381a, 0xffa8351a, 0xffa7321a, 0xffa5301a,
        0xffa42d1b, 0xffa22a1b, 0xffa0271b, 0xff9f251b, 0xff9d221b, 0xff9c1f1b, 0xff9a1c1c, 0xff98191c,
        0xff97171c, 0xff95141c, 0xff94111c, 0xff920e1c, 0xff910c1d, 0xff8f091d, 0xff8e061d, 0xff8c031d
    };

MainWindow7::MainWindow7(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow7)
    , m_scan(nullptr)
    , m_isScanning(false)
    , cudaProcessor(nullptr)
{
    ui->setupUi(this);
    // 统一界面字体大小为 14
    this->setStyleSheet(QStringLiteral("font-size: 14px;"));
    qRegisterMetaType<ScanFrame>();

    initWidget(); // 初始化界面

    initVTK();    // 初始化VTK

    // 初始化 Scan 对象
    m_scan = new Scan;
    m_scan->moveToThread(&m_scanThread);
    // 扫描自然结束/停止时同步 UI 状态
    connect(m_scan, &Scan::finished, this, [this]() {
        m_isScanning = false;
        m_drawPaused = false;
        ui->pushButton_7->setText("开始扫描");
        // 扫描自然结束时停止计时
        if (m_scanTimeRunning) {
            m_scanElapsedMs += m_scanTimer.elapsed();
            m_scanTimeRunning = false;
        }
        // 调试：扫描结束后输出网格带结构，检查带间是否有空隙
        dumpGridStats();
    });
    m_scanThread.start();

    // 每秒刷新左上角叠加层：数据帧率 / 渲染帧率 / 扫描时间 / 点云点数
    m_overlayTimer = new QTimer(this);
    m_overlayTimer->setInterval(1000);
    connect(m_overlayTimer, &QTimer::timeout, this, &MainWindow7::updateOverlay);
    m_overlayTimer->start();

    // 延迟注册 VBO (等待 OpenGL 上下文就绪)
    QTimer::singleShot(200, this, &MainWindow7::registerVBOWithCUDA);
}

void MainWindow7::autoStartDebug(const QString& csvPath)
{
    if (!m_scan || !m_scan->loadCSVFile(csvPath)) {
        qDebug() << "autoStartDebug: failed to load" << csvPath;
        return;
    }
    initPointCloud();
    disconnect(m_scan, &Scan::newFrameAvailable, this, &MainWindow7::renderFrame);
    connect(m_scan, &Scan::newFrameAvailable, this, &MainWindow7::renderFrame);
    QMetaObject::invokeMethod(m_scan, [this]() { m_scan->start(); }, Qt::QueuedConnection);
    m_isScanning = true;
    // 自动调试启动：重新计时
    m_scanElapsedMs = 0;
    m_scanTimer.restart();
    m_scanTimeRunning = true;
    qDebug() << "autoStartDebug started";
    QTimer::singleShot(10000, this, [this]() { on_pushButton_12_clicked(); });
    QTimer::singleShot(25000, this, [this]() {
        vtkIdType total = 0;
        for (auto& ch : m_surfChunks) total += ch.cells->GetNumberOfCells();
        qDebug() << "DBG5 chunks" << m_surfChunks.size() << "tri" << total << "gridRows" << m_surfGrid.size();
    });
}

MainWindow7::~MainWindow7()
{
    // 注销 CUDA VBO
    if (cudaProcessor) {
        unregisterVBO(cudaProcessor);
        destroyCUDAProcessor(cudaProcessor);
        cudaProcessor = nullptr;
    }

    // 释放 VBO
    if (vboPoints) {
        vboPoints->destroy();
        delete vboPoints;
        vboPoints = nullptr;
    }
    if (vboColors) {
        vboColors->destroy();
        delete vboColors;
        vboColors = nullptr;
    }

    if (m_scan) {
        QMetaObject::invokeMethod(m_scan, [this]() { m_scan->stop(); }, Qt::BlockingQueuedConnection);
    }
    if (m_scan) {
        QMetaObject::invokeMethod(m_scan, [this]() { m_scan->stop(); }, Qt::BlockingQueuedConnection);
        m_scan->deleteLater();
        m_scan = nullptr;
    }
    m_scanThread.quit();
    m_scanThread.wait();

    delete ui;
}

bool MainWindow7::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == ui->colorBarWidget && event->type() == QEvent::Paint) {

        QWidget* colorBar = ui->colorBarWidget;
        QPainter painter(colorBar);

        int width = colorBar->width();
        int height = colorBar->height();

        QLinearGradient gradient(0, 0, 0, height);

        const int steps = 256;
        for (int i = 0; i <= steps; i++) {
            double value = 1.0 - static_cast<double>(i) / steps;
            unsigned char r, g, b;
            GetColorFromValue(value, r, g, b);
            gradient.setColorAt(static_cast<double>(i) / steps, QColor(r, g, b));
        }

        painter.fillRect(0, 0, width, height, gradient);

        return true;
    }

    if (obj == ui->vtkWidget && (event->type() == QEvent::Resize || event->type() == QEvent::Move)) {
        positionOverlayLabel();
    }

    return QMainWindow::eventFilter(obj, event);
}

// 初始化界面
void MainWindow7::initWidget()
{
    // 自适应窗口
    this->centralWidget()->setLayout(ui->gridLayout_2);

    // 安装事件过滤器到colorBarWidget
    ui->colorBarWidget->installEventFilter(this);

}

// 初始化VTK
void MainWindow7::initVTK()
{
    // 创建渲染器
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0, 0, 0);

    // remove default frustum cullers so the custom VBO actor is always rendered
    while (renderer->GetCullers()->GetNumberOfItems() > 0) {
        renderer->RemoveCuller(renderer->GetCullers()->GetLastItem());
    }

    // 获取渲染窗口
    renderWindow = ui->vtkWidget->renderWindow();
    renderWindow->AddRenderer(renderer);

    // 设置交互样式
    style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
    interactor = renderWindow->GetInteractor();
    interactor->SetInteractorStyle(style);
    interactor->Initialize();

    // 添加鼠标回调
    vtkSmartPointer<vtkCallbackCommand> mouseCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    mouseCallback->SetCallback(onMouseClick);
    mouseCallback->SetClientData(this);
    interactor->AddObserver(vtkCommand::LeftButtonPressEvent, mouseCallback);

    // remember when the user rotates/zooms so auto-fit pauses during interaction
    vtkSmartPointer<vtkCallbackCommand> interactionCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    interactionCallback->SetCallback([](vtkObject*, unsigned long, void* cd, void*) {
        MainWindow7* self = static_cast<MainWindow7*>(cd);
        self->m_lastInteractionMs = QDateTime::currentMSecsSinceEpoch();
    });
    interactionCallback->SetClientData(this);
    interactor->AddObserver(vtkCommand::InteractionEvent, interactionCallback);

    // 交互（旋转/缩放/平移）时点云降采样，松开后恢复全量，解决大点云卡顿
    vtkSmartPointer<vtkCallbackCommand> startInteract = vtkSmartPointer<vtkCallbackCommand>::New();
    startInteract->SetCallback([](vtkObject*, unsigned long, void* cd, void*) {
        MainWindow7* self = static_cast<MainWindow7*>(cd);
        if (self->vboActor && self->cloudValidPoints > 20000) {
            // 交互时抽稀显示，但保持整片点云覆盖，而不是只画前 1/3 个点
            self->vboActor->SetDisplayStride(3);
        }
    });
    startInteract->SetClientData(this);
    interactor->AddObserver(vtkCommand::StartInteractionEvent, startInteract);

    vtkSmartPointer<vtkCallbackCommand> endInteract = vtkSmartPointer<vtkCallbackCommand>::New();
    endInteract->SetCallback([](vtkObject*, unsigned long, void* cd, void*) {
        MainWindow7* self = static_cast<MainWindow7*>(cd);
        if (self->vboActor)
            self->vboActor->SetDisplayStride(1);
    });
    endInteract->SetClientData(this);
    interactor->AddObserver(vtkCommand::EndInteractionEvent, endInteract);

    // 第一个点标记（红色）
    vtkSmartPointer<vtkSphereSource> sphere1 = vtkSmartPointer<vtkSphereSource>::New();
    sphere1->SetRadius(0.5);
    sphere1->SetThetaResolution(20);
    sphere1->SetPhiResolution(20);
    vtkSmartPointer<vtkPolyDataMapper> mapper1 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper1->SetInputConnection(sphere1->GetOutputPort());
    point1Actor = vtkSmartPointer<vtkActor>::New();
    point1Actor->SetMapper(mapper1);
    point1Actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    point1Actor->VisibilityOff();
    renderer->AddActor(point1Actor);

    // 第二个点标记（绿色）
    vtkSmartPointer<vtkSphereSource> sphere2 = vtkSmartPointer<vtkSphereSource>::New();
    sphere2->SetRadius(0.5);
    sphere2->SetThetaResolution(20);
    sphere2->SetPhiResolution(20);
    vtkSmartPointer<vtkPolyDataMapper> mapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper2->SetInputConnection(sphere2->GetOutputPort());
    point2Actor = vtkSmartPointer<vtkActor>::New();
    point2Actor->SetMapper(mapper2);
    point2Actor->GetProperty()->SetColor(0.0, 1.0, 2.0);
    point2Actor->VisibilityOff();
    renderer->AddActor(point2Actor);

    // 设置相机
    camera = renderer->GetActiveCamera();
    camera->SetPosition(0, -800, -200);
    camera->SetViewUp(0, 0, -1);
    camera->SetViewAngle(30);
    camera->SetClippingRange(-1000, 10000);

    // 设置灯光
    lightKit = vtkSmartPointer<vtkLightKit>::New();
    lightKit->SetKeyLightIntensity(5);
    lightKit->AddLightsToRenderer(renderer);
    renderer->SetAmbient(0.5, 0.5, 0.5);

    // 左上角实时状态叠加层（Qt UI 控件，悬浮在 VTK 窗口之上，不参与鼠标事件）
    m_overlayLabel = new QLabel(ui->vtkWidget->parentWidget());
    m_overlayLabel->setObjectName("overlayLabel");
    m_overlayLabel->setStyleSheet("background-color: rgba(0, 0, 0, 150);"
                                  " color: rgb(255, 255, 0);"
                                  " padding: 6px 10px;"
                                  " border-radius: 4px;"
                                  " font-size: 14px;");
    m_overlayLabel->setText("数据帧率: 0 fps\n渲染帧率: 0 fps\n扫描时间: 00:00:00\n点云点数: 0");
    m_overlayLabel->setAttribute(Qt::WA_TransparentForMouseEvents);
    m_overlayLabel->show();
    ui->vtkWidget->installEventFilter(this);
    positionOverlayLabel();

    renderer->ResetCamera();

    renderWindow->Render();
}

// 初始化点云
void MainWindow7::initPointCloud()
{
    savedAmpValues.clear();
    savedTofValues.clear();

    // 创建占位 PolyData 和 Mapper（确保 mapper 非空且长期存在）
    points   = vtkSmartPointer<vtkPoints>::New();
    vertices = vtkSmartPointer<vtkCellArray>::New();
    colors   = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetVerts(vertices);
    polyData->GetPointData()->SetScalars(colors);

    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    if (vboActor) {
        renderer->RemoveActor(vboActor);
        vboActor = nullptr;
    }
    vboActor = vtkSmartPointer<vtkVBOActor>::New();
    vboActor->VisibilityOn();
    renderer->AddActor(vboActor);
    m_surfChunks.clear();
    newSurfaceChunk();
    qDebug() << "initPointCloud: vboActor added";
    cloudValidPoints = 0;
    cloudBoundsValid = false;
    cameraFitValid = false;
    if (vboActor) vboActor->SetValidPointCount(0);
    if (cudaProcessor && vboRegistered) resetVBO(cudaProcessor);

    renderer->ResetCamera();

    renderWindow->Render();
}

void MainWindow7::registerVBOWithCUDA()
{
    if (!isCUDAAvailable()) {
        qDebug() << "CUDA not available";
        return;
    }

    if (!cudaProcessor) {
        cudaProcessor = createCUDAProcessor(MAX_BEAM);
        if (!cudaProcessor) {
            qDebug() << "Failed to create CUDA processor";
            return;
        }
    }

    // 确保 OpenGL 上下文
    if (!ui->vtkWidget) {
        qDebug() << "vtkWidget is null!";
        return;
    }

    ui->vtkWidget->makeCurrent();

    // 创建 VBO
    vboMaxPoints = MAX_POINTS_PER_FRAME;

    vboPoints = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    vboPoints->create();
    vboPoints->bind();
    vboPoints->allocate(vboMaxPoints * 3 * sizeof(float));
    vboPoints->release();

    vboColors = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    vboColors->create();
    vboColors->bind();
    vboColors->allocate(vboMaxPoints * 3 * sizeof(unsigned char));
    vboColors->release();

    qDebug() << "VBO Points ID:" << vboPoints->bufferId();
    qDebug() << "VBO Colors ID:" << vboColors->bufferId();

    // 注册 VBO 到 CUDA
    int result = registerVBO(cudaProcessor,
                             vboPoints->bufferId(),
                             vboColors->bufferId(),
                             vboMaxPoints);

    if (result == 0) {
        qDebug() << "VBO registered to CUDA successfully!";
        vboInitialized = true;
        vboRegistered = true;   // 置位：AMP/TOF 切换时才能对已有点云整体重着色
        setColorLUT(cudaProcessor, color_Amplitude, 256);
        cloudValidPoints = 0;
    } else {
        qDebug() << "Failed to register VBO to CUDA";
        vboRegistered = false;
        delete vboPoints;
        vboPoints = nullptr;
        delete vboColors;
        vboColors = nullptr;
    }
}

void MainWindow7::AddPointCloud(const double pose[], const double amp[],
                                const double tof[], double si, int beam,
                                bool renderNow, const double* beamZ,
                                const float* worldXYZ, int worldCount,
                                const int* gridK, const int* gridJ, int passIndex,
                                double lx, double ly)
{
    if (beam <= 0) return;
    if (!isCUDAAvailable()) return;
    if (!vboPoints || !vboColors || !vboActor) return;

    if (!cudaProcessor) {
        cudaProcessor = createCUDAProcessor(MAX_BEAM);
        if (!cudaProcessor) return;
        setColorLUT(cudaProcessor, color_Amplitude, 256);
    }

    if (cloudValidPoints >= MAX_POINTS_PER_FRAME) return;

    int n = worldXYZ ? worldCount : beam;
    if (n > MAX_BEAM) n = MAX_BEAM;

    int validCount = 0;
    for (int i = 0; i < n; i++) {
        if (amp[i] != 0.0 && tof[i] != 0.0) {
            validCount++;
            if ((int)savedAmpValues.size() < MAX_POINTS_PER_FRAME) {
                savedAmpValues.push_back(amp[i]);
                savedTofValues.push_back(tof[i]);
                if (gridK && gridJ) {
                    m_gridK.push_back(gridK[i]); m_gridJ.push_back(gridJ[i]);
                    m_surfPointPass.push_back(passIndex);
                }
                if (worldXYZ) {
                    m_cloudXYZ.push_back(worldXYZ[i*3+0]);
                    m_cloudXYZ.push_back(worldXYZ[i*3+1]);
                    m_cloudXYZ.push_back(worldXYZ[i*3+2]);
                }
            }
        }
    }
    if (validCount > 0) {
        std::array<double, 6> pp = {pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]};
        m_framePose.push_back(pp);
        m_frameSi.push_back(si);
        m_frameBeam.push_back(beam);
        m_frameLX.push_back(lx);
        m_frameLY.push_back(ly);
        m_frameCount.push_back(validCount);
    }
    if (validCount == 0) return;

    if (worldXYZ) {
        for (int i = 0; i < n; i++) {
            double px = worldXYZ[i*3+0], py = worldXYZ[i*3+1], pz = worldXYZ[i*3+2];
            if (!cloudBoundsValid || px < cloudBoundsMin[0]) cloudBoundsMin[0] = px;
            if (!cloudBoundsValid || px > cloudBoundsMax[0]) cloudBoundsMax[0] = px;
            if (!cloudBoundsValid || py < cloudBoundsMin[1]) cloudBoundsMin[1] = py;
            if (!cloudBoundsValid || py > cloudBoundsMax[1]) cloudBoundsMax[1] = py;
            if (!cloudBoundsValid || pz < cloudBoundsMin[2]) cloudBoundsMin[2] = pz;
            if (!cloudBoundsValid || pz > cloudBoundsMax[2]) cloudBoundsMax[2] = pz;
        }
    } else {
        for (int k = 0; k < 3; k++) {
            if (!cloudBoundsValid || pose[k] < cloudBoundsMin[k]) cloudBoundsMin[k] = pose[k];
            if (!cloudBoundsValid || pose[k] > cloudBoundsMax[k]) cloudBoundsMax[k] = pose[k];
        }
    }
    cloudBoundsValid = true;
    vboActor->SetCloudBounds(cloudBoundsMin[0] - 20, cloudBoundsMax[0] + 20,
                            cloudBoundsMin[1] - 20, cloudBoundsMax[1] + 20,
                            cloudBoundsMin[2] - 20, cloudBoundsMax[2] + 20);

    int result;
    if (worldXYZ) {
        result = processDirectCloudVBO(cudaProcessor, worldXYZ, amp, tof, n,
                                       isAmpMode ? 1 : 0, cloudValidPoints);
    } else {
        double localZ[64];
        for (int i = 0; i < beam; i++)
            localZ[i] = beamZ ? beamZ[i] : -si * 0.5;
        result = processDirectVBO(cudaProcessor, pose, amp, tof, localZ, beam,
                                  isAmpMode ? 1 : 0, cloudValidPoints);
    }

    if (result == 0) {
        cloudValidPoints += validCount;
        if (cloudValidPoints > MAX_POINTS_PER_FRAME) cloudValidPoints = MAX_POINTS_PER_FRAME;

        vboActor->SetVBOIDs(vboPoints->bufferId(), vboColors->bufferId());
        vboActor->SetValidPointCount(cloudValidPoints);
        // 显示抽稀：点云超过显示预算时，只绘制下标为 stride 倍数的点，
        // 数据完整保留在 VBO/内存里，仅控制每帧绘制量，保证渲染帧率稳定。
        // 步长取 2 的幂且只增不减，避免抽稀图案在云增长时跳动。
        while (cloudValidPoints / m_displayStride > 1200000 && m_displayStride < 64)
            m_displayStride *= 2;
        vboActor->SetDisplayStride(m_displayStride);
        vboActor->SetVBOInitialized(true);

        { // auto-fit camera to the growing cloud (throttled)
            static QElapsedTimer fitTimer;
            if (!fitTimer.isValid()) fitTimer.start();
            bool firstFit = !cameraFitValid;
            bool outOfFit = !firstFit &&
                (cloudBoundsMin[0] < cameraFitMin[0] || cloudBoundsMax[0] > cameraFitMax[0] ||
                 cloudBoundsMin[1] < cameraFitMin[1] || cloudBoundsMax[1] > cameraFitMax[1] ||
                 cloudBoundsMin[2] < cameraFitMin[2] || cloudBoundsMax[2] > cameraFitMax[2]);
            bool userActive = QDateTime::currentMSecsSinceEpoch() - m_lastInteractionMs < 3000;
            if (!userActive && (firstFit || outOfFit) && (firstFit || fitTimer.elapsed() >= 1500)) {
                double cloudBounds[6] = {
                    cloudBoundsMin[0] - 20, cloudBoundsMax[0] + 20,
                    cloudBoundsMin[1] - 20, cloudBoundsMax[1] + 20,
                    cloudBoundsMin[2] - 20, cloudBoundsMax[2] + 20
                };
                if (firstFit) {
                    double cx = (cloudBounds[0] + cloudBounds[1]) * 0.5;
                    double cy = (cloudBounds[2] + cloudBounds[3]) * 0.5;
                    double cz = (cloudBounds[4] + cloudBounds[5]) * 0.5;
                    camera->SetFocalPoint(cx, cy, cz);
                    camera->SetPosition(cx, cy, cz + 3000);
                    camera->SetViewUp(0, 1, 0);
                }
                renderer->ResetCamera(cloudBounds);
                for (int k = 0; k < 3; k++) {
                    cameraFitMin[k] = cloudBoundsMin[k];
                    cameraFitMax[k] = cloudBoundsMax[k];
                }
                cameraFitValid = true;
                fitTimer.restart();
            }
        }

        if (renderNow) {
            renderWindow->Render();

        }
    } else if (result != -2) {
        qDebug() << "VBO Direct failed:" << result;
    }
}

void MainWindow7::resetPointCloud()
{
    cloudValidPoints = 0;
    m_displayStride = 1;
    cloudBoundsValid = false;
    cameraFitValid = false;
    savedAmpValues.clear();
    savedTofValues.clear();
    m_gridK.clear();
    m_gridJ.clear();
    m_surfPointPass.clear();
    m_framePose.clear();
    m_frameSi.clear();
    m_frameBeam.clear();
    m_frameLX.clear();
    m_frameLY.clear();
    m_frameCount.clear();
    m_cloudXYZ.clear();
    m_meshBuiltCount = 0;
    m_meshIdx.clear();
    m_meshQuadVerts.clear();
    m_meshVertIdx.clear();
    m_meshPatchDone.clear();
    m_surfGrid.clear();
    m_surfDataRows.clear();
    m_surfRowCnt.clear();
    m_surfRowDone.clear();
    m_surfPrevRow.clear();
    m_surfHasPrev = false;
    m_surfPrevKey = -1;
    m_surfLastDataJ = 0;
    m_surfLastDir = 0;
    m_surfDirInit = false;
    m_surfDataN = 0;
    m_meshCurBand = -1;
    m_surfRowLastSeen.clear();
    for (auto& ch : m_surfChunks) if (ch.actor) ch.actor->VisibilityOff();
    m_surfChunks.clear();
    newSurfaceChunk();
    m_surfaceMode = false;
    if (cudaProcessor && vboRegistered) {
        resetVBO(cudaProcessor);
    }
    if (vboActor) {
        vboActor->SetValidPointCount(0);
        vboActor->SetVBOInitialized(false);
    }
    renderWindow->Render();
}

void MainWindow7::renderFrame(const ScanFrame& frame)
{
    // 添加点云
    static QElapsedTimer lastRender;
    if (!lastRender.isValid()) lastRender.start();
    bool renderNow = lastRender.elapsed() >= 33;
    if (renderNow) lastRender.restart();
    AddPointCloud(frame.pose, frame.amp, frame.tof, frame.si, frame.beam, renderNow, frame.beamZ,
                        frame.hasWorld ? frame.worldXYZ : nullptr, frame.hasWorld ? frame.worldCount : 0,
                        frame.hasGrid ? frame.gridK : nullptr, frame.hasGrid ? frame.gridJ : nullptr,
                        frame.passIndex, frame.lx, frame.ly);
    {
        static int sTick = 0;
        // 降低曲面网格重建频率，减轻 GUI 线程负载（约 5 次/秒）
        if (++sTick % 40 == 0) updateSurfaceMesh(frame.passIndex);
    }

    // 统计每秒收到的数据帧数与实际渲染帧数（左上角叠加层每秒读取并清零）
    m_guiFrames++;
    if (renderNow) m_renderCount++;

    // 保存最新一帧关键信息，供叠加层/UI 行每秒刷新
    m_lastIpoc = frame.ipoc;
    m_lastSi = frame.si;
    m_lastBeam = frame.beam;
    m_lastAmp0 = frame.amp[0];
    m_lastTof0 = frame.tof[0];
    for (int k = 0; k < 6; k++) m_lastPose[k] = frame.pose[k];
}

// 每秒刷新左上角叠加层与 UI 数据行
void MainWindow7::updateOverlay()
{
    int guiFrames = m_guiFrames;
    int renderFps = m_renderCount;
    m_guiFrames = 0;
    m_renderCount = 0;
    qDebug() << "[PointCloud] GUI收到帧:" << guiFrames
             << " 渲染帧率:" << renderFps << "fps"
             << " 点云点数:" << cloudValidPoints
             << " 保存点数:" << savedAmpValues.size();

    if (m_overlayLabel) {
        qint64 scanMs = m_scanElapsedMs + (m_scanTimeRunning ? m_scanTimer.elapsed() : 0);
        QString scanTime = QTime::fromMSecsSinceStartOfDay(scanMs).toString("hh:mm:ss");
        QString fpsText = QString("数据帧率: %1 fps\n渲染帧率: %2 fps\n扫描时间: %3\n点云点数: %4")
                              .arg(guiFrames)
                              .arg(renderFps)
                              .arg(scanTime)
                              .arg(cloudValidPoints);
        m_overlayLabel->setText(fpsText);
        positionOverlayLabel();
    }

    // 打印实时数据到 UI（最新一帧）
    QString text = QString("Frame: %1 SI: %2 Beam: %3 "
                           "Pose: %4 %5 %6 %7 %8 %9 "
                           "AMP[0]: %10 TOF[0]: %11")
                       .arg(m_lastIpoc)
                       .arg(m_lastSi, 0, 'f', 2)
                       .arg(m_lastBeam)
                       .arg(m_lastPose[0], 0, 'f', 2)
                       .arg(m_lastPose[1], 0, 'f', 2)
                       .arg(m_lastPose[2], 0, 'f', 2)
                       .arg(m_lastPose[3], 0, 'f', 2)
                       .arg(m_lastPose[4], 0, 'f', 2)
                       .arg(m_lastPose[5], 0, 'f', 2)
                       .arg(m_lastAmp0, 0, 'f', 2)
                       .arg(m_lastTof0, 0, 'f', 2);
    ui->lineEdit->setText(text);
    ui->lineEdit->setCursorPosition(0);
}

// 将叠加层定位到 VTK 窗口左上角
void MainWindow7::positionOverlayLabel()
{
    if (!m_overlayLabel || !ui->vtkWidget) return;
    QWidget* host = ui->vtkWidget->parentWidget();
    if (!host) return;
    m_overlayLabel->adjustSize();
    QPoint topLeft = ui->vtkWidget->mapTo(host, QPoint(0, 0));
    m_overlayLabel->move(topLeft + QPoint(8, 8));
    m_overlayLabel->raise();
}

void MainWindow7::onMouseClick(vtkObject* obj, unsigned long, void* clientData, void*)
{
    MainWindow7* self = static_cast<MainWindow7*>(clientData);
    if (!self->isMeasuring) return;

    vtkRenderWindowInteractor* interactor =
        static_cast<vtkRenderWindowInteractor*>(obj);

    int x = interactor->GetEventPosition()[0];
    int y = interactor->GetEventPosition()[1];

    self->pickPoint(x, y);
}

void MainWindow7::pickPoint(int x, int y)
{
    if (!points || points->GetNumberOfPoints() == 0 || !isMeasuring) return;

    // 拾取点
    vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
    picker->SetTolerance(0.00015);
    picker->Pick(x, y, 0, renderer);

    // 获取点击位置和选中点
    double pickPos[3], p[3];
    picker->GetPickPosition(pickPos);
    vtkIdType pid = picker->GetPointId();
    points->GetPoint(pid, p);

    // 检查是否点击到有效点
    double dx = p[0] - pickPos[0], dy = p[1] - pickPos[1], dz = p[2] - pickPos[2];
    if (dx*dx + dy*dy + dz*dz > 0.00010) {
        return;
    }

    // 更新当前点（红色）
    point1Actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    point1Actor->SetPosition(p);
    point1Actor->VisibilityOn();

    // 计算与上一个点的距离
    double dist = 0.0;
    if (hasLastPoint) {
        point2Actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
        point2Actor->SetPosition(lastPoint);
        point2Actor->VisibilityOn();
        dist = sqrt(pow(p[0]-lastPoint[0], 2) +
                    pow(p[1]-lastPoint[1], 2) +
                    pow(p[2]-lastPoint[2], 2));
    }

    // 更新状态栏
    ui->lineEdit->setText(QString("AMP=%1  TOF=%2  Distance=%3")
                                   .arg(savedAmpValues[pid], 0, 'f', 3)
                                   .arg(savedTofValues[pid], 0, 'f', 3)
                                   .arg(dist, 0, 'f', 3));

    // 保存当前点为上一个点
    lastPoint[0] = p[0]; lastPoint[1] = p[1]; lastPoint[2] = p[2];
    hasLastPoint = true;
    renderWindow->Render();
}

void MainWindow7::GetColorFromValue(double value, unsigned char& r, unsigned char& g, unsigned char& b)
{
    value = std::max(0.0, std::min(1.0, value));

    const int colorCount = sizeof(color_Amplitude) / sizeof(uint32_t);
    int index = static_cast<int>(value * (colorCount - 1));
    index = std::max(0, std::min(colorCount - 1, index));

    uint32_t color = color_Amplitude[index];
    r = static_cast<unsigned char>((color >> 16) & 0xFF);
    g = static_cast<unsigned char>((color >> 8) & 0xFF);
    b = static_cast<unsigned char>(color & 0xFF);
}

// 开始绘制
void MainWindow7::on_pushButton_clicked()
{
    // 仅全新开始（未扫描且非暂停）时重置绘制数据；
    // 暂停后继续保留数据；已在扫描时不重置
    if (!vboActor) {
        // Drawing was never initialized (e.g. the user clicked "start scan"
        // before "start drawing"): initialize it even though the scan is
        // already running.
        initPointCloud();
    } else if (!m_isScanning && !m_drawPaused) {
        initPointCloud();
    }

    disconnect(m_scan, &Scan::newFrameAvailable, this, &MainWindow7::renderFrame);
    connect(m_scan, &Scan::newFrameAvailable, this, &MainWindow7::renderFrame);

    if (!m_isScanning) {
        QMetaObject::invokeMethod(m_scan, [this]() { m_scan->start(); }, Qt::QueuedConnection);
        m_isScanning = true;
        ui->pushButton_7->setText("停止扫描");
        qDebug() << "Scan started for rendering";
        // 扫描计时：全新开始清零，暂停后续扫不清零
        if (!m_drawPaused) m_scanElapsedMs = 0;
        m_scanTimer.restart();
        m_scanTimeRunning = true;
    } else {
        qDebug() << "Drawing connected (scan already running)";
        if (!m_scanTimeRunning) {
            m_scanTimer.restart();
            m_scanTimeRunning = true;
        }
    }
    m_drawPaused = false;
}

// 停止绘制（暂停，可继续）
void MainWindow7::on_pushButton_2_clicked()
{
    if (m_isScanning) {
        QMetaObject::invokeMethod(m_scan, [this]() { m_scan->pause(); }, Qt::QueuedConnection);
        m_isScanning = false;
        m_drawPaused = true;
        ui->pushButton_7->setText("开始扫描");
        qDebug() << "Drawing paused";
        // 暂停时冻结扫描时间
        if (m_scanTimeRunning) {
            m_scanElapsedMs += m_scanTimer.elapsed();
            m_scanTimeRunning = false;
        }
    }
}

// 结束绘制
void MainWindow7::on_pushButton_3_clicked()
{
    if (m_isScanning) {
        QMetaObject::invokeMethod(m_scan, [this]() { m_scan->stop(); }, Qt::QueuedConnection);
        m_isScanning = false;
        m_drawPaused = false;
        ui->pushButton_7->setText("开始扫描");
    }
    // 结束绘制时停止计时
    if (m_scanTimeRunning) {
        m_scanElapsedMs += m_scanTimer.elapsed();
        m_scanTimeRunning = false;
    }
    // 收尾剩余行，形成完整曲面
    updateSurfaceMesh(m_meshCurBand, true);
    renderWindow->Render();
    QTimer::singleShot(800, this, [this]() {
        updateSurfaceMesh(m_meshCurBand, true);
        renderWindow->Render();
    });
    qDebug() << "Drawing finished";
}

// SoundScan 扫描控制链接入口
void MainWindow7::startDrawing()
{
    on_pushButton_clicked();
}

void MainWindow7::pauseDrawing()
{
    on_pushButton_2_clicked();
}

void MainWindow7::finishDrawing()
{
    on_pushButton_3_clicked();
}

// 调试：输出点云网格的带(k)/行(j)结构，定位带间空隙
void MainWindow7::dumpGridStats()
{
    int n = (int)m_gridK.size();
    if (n == 0 || (int)m_surfPointPass.size() < n || (int)m_gridJ.size() < n) {
        qDebug() << "[GridStats] 无网格数据";
        return;
    }
    int maxPass = 0;
    for (int i = 0; i < n; i++) {
        if (m_surfPointPass[i] > maxPass) maxPass = m_surfPointPass[i];
    }
    qDebug() << "[GridStats] 总点数:" << n << " 带数:" << (maxPass + 1);
    for (int p = 0; p <= maxPass; p++) {
        long long minK = 0, maxK = 0, minJ = 0, maxJ = 0;
        int cnt = 0;
        bool first = true;
        for (int i = 0; i < n; i++) {
            if (m_surfPointPass[i] != p) continue;
            long long k = m_gridK[i];
            long long j = m_gridJ[i];
            if (first) {
                minK = maxK = k;
                minJ = maxJ = j;
                first = false;
            } else {
                if (k < minK) minK = k;
                if (k > maxK) maxK = k;
                if (j < minJ) minJ = j;
                if (j > maxJ) maxJ = j;
            }
            cnt++;
        }
        qDebug() << "[GridStats] 带" << p << ": k=[" << minK << "," << maxK << "]"
                 << " j=[" << minJ << "," << maxJ << "] 点数:" << cnt;
    }
}

// 保存数据
void MainWindow7::on_pushButton_5_clicked()
{
    int n = (int)m_frameCount.size();
    if (n == 0) {
        QMessageBox::information(this, "保存数据", "当前没有可保存的数据");
        return;
    }
    QString filename = QFileDialog::getSaveFileName(
        this, "保存数据", "", "CSV Files (*.csv);;All Files (*)");
    if (filename.isEmpty()) return;
    if (!filename.endsWith(".csv", Qt::CaseInsensitive)) filename += ".csv";
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(this, "错误", "无法写入文件: " + filename);
        return;
    }
    QTextStream out(&file);
    out.setRealNumberPrecision(6);
    // 与导入的 CSV 格式保持一致
    out << "X,Y,Z,A,B,C,SI";
    for (int e = 1; e <= 49; e++) out << ",AMP_" << e << ",TOF_" << e;
    out << ",BEAM,LX,LY\n";

    int totalPts = (int)savedAmpValues.size();
    int p = 0;
    for (int f = 0; f < n; f++) {
        const std::array<double, 6>& pp = m_framePose[f];
        out << pp[0] << ',' << pp[1] << ',' << pp[2] << ','
            << pp[3] << ',' << pp[4] << ',' << pp[5] << ','
            << m_frameSi[f] << ',';
        int cnt = m_frameCount[f];
        for (int e = 0; e < 49; e++) {
            if (e < cnt && p + e < totalPts)
                out << savedAmpValues[p + e] << ',' << savedTofValues[p + e];
            else
                out << "0,0";
            if (e < 48) out << ',';
        }
        out << ',' << m_frameBeam[f] << ',' << m_frameLX[f] << ',' << m_frameLY[f] << '\n';
        p += cnt;
    }
    qDebug() << "Saved" << n << "frames to" << filename;
}

// 加载数据
void MainWindow7::on_pushButton_6_clicked()
{
    // 初始化点云
    initPointCloud();

    // 弹出文件选择对话框
    QString filename = QFileDialog::getOpenFileName(
        this,
        "选择数据文件",
        "",
        "CSV Files (*.csv);;All Files (*)"
        );

    if (filename.isEmpty()) {
        return;
    }

    std::ifstream file(filename.toStdString());
    if (!file.is_open()) {
        QMessageBox::warning(this, "错误", "无法打开文件: " + filename);
        return;
    }

    // 获取文件大小
    file.seekg(0, std::ios::end);
    std::streampos fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // 创建进度对话框
    QProgressDialog progressDialog("正在加载数据...", "取消", 0, 100, this);
    progressDialog.setWindowTitle("加载进度");
    progressDialog.setWindowModality(Qt::WindowModal);
    progressDialog.setMinimumDuration(0);
    progressDialog.setValue(0);

    // CSV解析函数
    auto parseCSVLine = [](const std::string& line) -> std::vector<std::string> {
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
    };

    // 安全转换函数
    auto safe_stod = [](const std::string& str, double defaultVal = 0.0) -> double {
        std::string trimmed = str;
        // 去除首尾空格
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
    };

    // 读取表头
    std::string headerLine;
    if (!std::getline(file, headerLine)) {
        QMessageBox::warning(this, "错误", "文件为空或缺少表头行");
        file.close();
        return;
    }

    // 记录表头位置
    std::streampos headerPos = file.tellg();

    std::vector<std::string> headers = parseCSVLine(headerLine);

    // 建立列名到索引的映射
    std::map<std::string, int> Index;
    for (int i = 0; i < (int)headers.size(); i++) {
        Index[headers[i]] = i;
    }

    // 根据列名获取索引
    auto get = [&](const std::string& Name) -> int {
        auto it = Index.find(Name);
        if (it != Index.end()) {
            return it->second;
        }
        return -1;
    };

    // 统计信息
    bool canceled = false;
    int beam = 0;
    double longmen[2] = {0.0};
    double si = 0.0;
    double amp[64] = {0.0};
    double tof[64] = {0.0};

    int BEAM  = get("BEAM");
    int LX    = get("LX");
    int LY    = get("LY");
    int X     = get("X");
    int SI    = get("SI");
    int AMP_1 = get("AMP_1");
    int TOF_1 = get("TOF_1");

    std::string prevLine;
    std::string line;
    int frameIndex = 0;

    while (std::getline(file, line))
    {
        if (progressDialog.wasCanceled()) {
            canceled = true;
            break;
        }

        std::streampos currentPos = file.tellg();
        if (currentPos >= headerPos && fileSize > headerPos) {
            double progress = static_cast<double>(currentPos - headerPos) /
                              static_cast<double>(fileSize - headerPos) * 100.0;
            progressDialog.setValue(static_cast<int>(progress));
        }

        if (frameIndex == 0) {
            prevLine = line;
            frameIndex++;
            continue;
        }

        std::vector<std::string> cells_main = parseCSVLine(prevLine);
        std::vector<std::string> cells_current = parseCSVLine(line);

        beam = 49;
        if (BEAM != -1 && BEAM < (int)cells_main.size()) {
            double bv = safe_stod(cells_main[BEAM], 49.0);
            if (bv > 0 && bv <= 64) beam = (int)bv;
        }

        if (LX != -1 && LX < (int)cells_main.size()) longmen[0] = safe_stod(cells_main[LX]);
        if (LY != -1 && LY < (int)cells_main.size()) longmen[1] = safe_stod(cells_main[LY]);

        double pose[6] = {0, 0, 0, 0, 0, 0};
        if (X != -1) {
            for (int i = 0; i < 6 && (X + i) < (int)cells_main.size(); i++) {
                pose[i] = safe_stod(cells_main[X + i]);
            }
        }
        pose[0] += longmen[1];
        pose[1] -= longmen[0];

        si = 0.0;
        if (SI != -1 && SI < (int)cells_current.size()) si = safe_stod(cells_current[SI]);

        for (int i = 0; i < beam; i++) {
            int ampIndex = (AMP_1 != -1 ? AMP_1 + 2 * i : -1);
            int tofIndex = (TOF_1 != -1 ? TOF_1 + 2 * i : -1);
            amp[i]  = (ampIndex >= 0 && ampIndex < (int)cells_current.size()) ? safe_stod(cells_current[ampIndex]) : 0.0;
            tof[i]  = (tofIndex >= 0 && tofIndex < (int)cells_current.size()) ? safe_stod(cells_current[tofIndex]) : 0.0;
        }

        AddPointCloud(pose, amp, tof, si, beam, false);

        prevLine = line;
        frameIndex++;
    }

    file.close();

    if (canceled) {
        progressDialog.setValue(100);
        return;
    }

    progressDialog.setValue(100);

    renderWindow->Render();
}

// 重置数据
void MainWindow7::on_pushButton_4_clicked()
{
    m_drawPaused = false;
    resetPointCloud();
}

// 数据模式
void MainWindow7::on_pushButton_8_clicked()
{
    // 先切换模式，保证点云(VBO)和曲面网格使用同一个新模式
    isAmpMode = !isAmpMode;

    // switch AMP/TOF color mode for the whole accumulated cloud
    const std::vector<double>& src = isAmpMode ? savedAmpValues : savedTofValues;
    int n = (int)src.size();
    if (n > cloudValidPoints) n = cloudValidPoints;

    if (n > 0 && cudaProcessor && vboRegistered) {
        std::vector<float> values(n);
        for (int i = 0; i < n; i++) values[i] = (float)src[i];
        ui->vtkWidget->makeCurrent();
        int rc = recolorVBO(cudaProcessor, values.data(), n);
        ui->vtkWidget->doneCurrent();
        qDebug() << "recolorVBO rc:" << rc << " points:" << n;
    }

    recolorSurfaceMesh();
    ui->pushButton_8->setText(isAmpMode ? "TOF\u6a21\u5f0f" : "AMP\u6a21\u5f0f");

    renderWindow->Render();
}

void MainWindow7::on_pushButton_10_clicked()
{
    isMeasuring = !isMeasuring;

    if (isMeasuring)
    {
        point1Actor->VisibilityOn();

        point2Actor->VisibilityOn();

        ui->pushButton_10->setText("退出测量");
    }
    else
    {
        point1Actor->VisibilityOff();

        point2Actor->VisibilityOff();

        ui->pushButton_10->setText("数据测量");
    }

    renderWindow->Render(); 
}

// 对齐视点
void MainWindow7::on_pushButton_9_clicked()
{
    vtkCamera* camera = renderer->GetActiveCamera();

    // 获取当前相机位置和焦点
    double focalPoint[3];
    camera->GetFocalPoint(focalPoint);

    double position[3];
    camera->GetPosition(position);

    // 计算从焦点到相机的向量
    double viewDir[3] = {
        position[0] - focalPoint[0],
        position[1] - focalPoint[1],
        position[2] - focalPoint[2]
    };

    // 归一化
    double length = sqrt(viewDir[0]*viewDir[0] + viewDir[1]*viewDir[1] + viewDir[2]*viewDir[2]);
    if (length > 0) {
        viewDir[0] /= length;
        viewDir[1] /= length;
        viewDir[2] /= length;
    }

    // 获取当前距离
    double distance = length;

    // 判断最近的方向（取绝对值最大的分量）
    double absX = fabs(viewDir[0]);
    double absY = fabs(viewDir[1]);
    double absZ = fabs(viewDir[2]);

    // 默认保持原方向
    double newPos[3] = {position[0], position[1], position[2]};
    double up[3] = {0, 0, 1};  // 默认上方向

    // 根据最大的分量确定视图方向
    if (absX >= absY && absX >= absZ) {
        // 对齐到X轴
        if (viewDir[0] > 0) {
            // 前视图 (X正方向)
            newPos[0] = focalPoint[0] + distance;
            newPos[1] = focalPoint[1];
            newPos[2] = focalPoint[2];
            up[0] = 0; up[1] = 0; up[2] = -1;
        } else {
            // 后视图 (X负方向)
            newPos[0] = focalPoint[0] - distance;
            newPos[1] = focalPoint[1];
            newPos[2] = focalPoint[2];
            up[0] = 0; up[1] = 0; up[2] = -1;
        }
    } else if (absY >= absX && absY >= absZ) {
        // 对齐到Y轴
        if (viewDir[1] > 0) {
            // 右视图 (Y正方向)
            newPos[0] = focalPoint[0];
            newPos[1] = focalPoint[1] + distance;
            newPos[2] = focalPoint[2];
            up[0] = 0; up[1] = 0; up[2] = -1;
        } else {
            // 左视图 (Y负方向)
            newPos[0] = focalPoint[0];
            newPos[1] = focalPoint[1] - distance;
            newPos[2] = focalPoint[2];
            up[0] = 0; up[1] = 0; up[2] = -1;
        }
    } else {
        // 对齐到Z轴
        if (viewDir[2] > 0) {
            // 俯视图 (Z正方向)
            newPos[0] = focalPoint[0];
            newPos[1] = focalPoint[1];
            newPos[2] = focalPoint[2] + distance;
            up[0] = 0; up[1] = 1; up[2] = 0;
        } else {
            // 仰视图 (Z负方向)
            newPos[0] = focalPoint[0];
            newPos[1] = focalPoint[1];
            newPos[2] = focalPoint[2] - distance;
            up[0] = 0; up[1] = 1; up[2] = 0;
        }
    }

    // 应用新的相机位置
    camera->SetPosition(newPos);
    camera->SetViewUp(up);
    camera->SetFocalPoint(focalPoint);

    renderer->ResetCameraClippingRange();

    renderWindow->Render();
}

// 窗口截图
void MainWindow7::on_pushButton_11_clicked()
{
    if (!renderWindow) {
        QMessageBox::warning(this, "错误", "渲染窗口未初始化");
        return;
    }

    // 强制渲染确保内容更新
    renderWindow->Render();

    // 截图
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
        vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->SetScale(1);  // 1为原始大小
    windowToImageFilter->SetInputBufferTypeToRGBA();
    windowToImageFilter->ReadFrontBufferOff();
    windowToImageFilter->Update();

    // 生成带时间戳的默认文件名
    QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString defaultFileName = QString("scan_%1.png").arg(timestamp);
    QString defaultPath = QDir::homePath() + "/" + defaultFileName;

    // 文件对话框（只显示PNG）
    QString filePath = QFileDialog::getSaveFileName(
        this,
        "保存截图",
        defaultPath,
        "PNG图片 (*.png)"
        );

    // 用户取消
    if (filePath.isEmpty()) {
        return;
    }

    // 确保扩展名为.png
    if (!filePath.endsWith(".png", Qt::CaseInsensitive)) {
        filePath += ".png";
    }

    // 保存为PNG
    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName(filePath.toStdString().c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
}

// 开始扫描
void MainWindow7::on_pushButton_7_clicked()
{
    if (!m_isScanning) {
        // ===== 开始扫描 =====

        // 检查是否已加载数据
        if (!m_scan->isFileLoaded()) {
            // 选择CSV文件
            QString filename = QFileDialog::getOpenFileName(
                this,
                "选择数据文件",
                "",
                "CSV Files (*.csv);;All Files (*)"
                );

            if (filename.isEmpty()) {
                return;
            }

            if (!m_scan->loadCSVFile(filename)) {
                QMessageBox::warning(this, "错误", "加载文件失败");
                return;
            }
        }

        // 启动数据采集
        QMetaObject::invokeMethod(m_scan, [this]() { m_scan->start(); }, Qt::QueuedConnection);
        m_isScanning = true;
        ui->pushButton_7->setText("停止扫描");

        qDebug() << "Scan started";

    } else {
        // ===== 停止扫描 =====
        QMetaObject::invokeMethod(m_scan, [this]() { m_scan->stop(); }, Qt::QueuedConnection);
        m_isScanning = false;
        m_drawPaused = false;
        ui->pushButton_7->setText("开始扫描");

        qDebug() << "Scan stopped";
    }
}

// 曲面模式 / 点云模式 切换
void MainWindow7::on_pushButton_12_clicked()
{
    m_surfaceMode = !m_surfaceMode;
    if (m_surfaceMode) {
        updateSurfaceMesh(m_meshCurBand);
        if (vboActor) vboActor->VisibilityOff();
        for (auto& ch : m_surfChunks) if (ch.actor) ch.actor->VisibilityOn();
        ui->pushButton_12->setText(QStringLiteral("点云模式"));
    } else {
        if (vboActor) vboActor->VisibilityOn();
        for (auto& ch : m_surfChunks) if (ch.actor) ch.actor->VisibilityOff();
        ui->pushButton_12->setText(QStringLiteral("曲面模式"));
    }
    renderWindow->Render();
}

void MainWindow7::newSurfaceChunk()
{
    SurfChunk c;
    c.points = vtkSmartPointer<vtkPoints>::New();
    c.cells = vtkSmartPointer<vtkCellArray>::New();
    c.cellColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    c.cellColors->SetNumberOfComponents(3);
    c.poly = vtkSmartPointer<vtkPolyData>::New();
    c.poly->SetPoints(c.points);
    c.poly->SetPolys(c.cells);
    c.poly->GetCellData()->SetScalars(c.cellColors);
    c.mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    c.mapper->SetInputData(c.poly);
    c.mapper->SetScalarModeToUseCellData();
    c.mapper->SetInterpolateScalarsBeforeMapping(0);
    c.actor = vtkSmartPointer<vtkActor>::New();
    c.actor->SetMapper(c.mapper);
    c.actor->GetProperty()->LightingOff();
    c.actor->SetVisibility(m_surfaceMode ? 1 : 0);
    renderer->AddActor(c.actor);
    m_surfChunks.push_back(c);
    m_meshVertIdx.clear();
}

void MainWindow7::updateSurfaceMesh(int passIndex, bool forceTail)
{
    int n = (int)m_gridK.size();
    if (n < 4 || (int)m_cloudXYZ.size() < n * 3) return;
    if (m_surfChunks.empty()) return;
    if (m_meshBuiltCount >= n && !forceTail) return;
    if (m_meshBuiltCount < 0) m_meshBuiltCount = 0;
    SurfChunk* cur = &m_surfChunks.back();
    if (cur->cells->GetNumberOfCells() >= kChunkMaxCells)
        { newSurfaceChunk(); cur = &m_surfChunks.back(); }

    auto gridHave = [&](int m, int k) -> bool {
        auto it = m_surfGrid.find(m);
        return it != m_surfGrid.end() && it->second.find(k) != it->second.end();
    };
    auto cornerVert = [&](int cm, int ck) -> int {
        long long ckey = ((long long)cm << 32) | (unsigned int)(ck & 0xFFFFFFFFLL);
        auto it = m_meshVertIdx.find(ckey);
        if (it != m_meshVertIdx.end()) return it->second;
        const SurfCell& cell = m_surfGrid[cm][ck];
        vtkIdType vid = cur->points->InsertNextPoint(cell.x, cell.y, cell.z);
        m_meshVertIdx[ckey] = (int)vid;
        return (int)vid;
    };
    auto createPatch = [&](int pm, int pk) {
        long long pkey = ((long long)pm << 32) | (unsigned int)(pk & 0xFFFFFFFFLL);
        if (m_meshPatchDone.find(pkey) != m_meshPatchDone.end()) return;
        if (!gridHave(pm, pk) || !gridHave(pm + 1, pk) ||
            !gridHave(pm + 1, pk + 1) || !gridHave(pm, pk + 1)) return;
        int sw = cornerVert(pm, pk);
        int se = cornerVert(pm + 1, pk);
        int ne = cornerVert(pm + 1, pk + 1);
        int nw = cornerVert(pm, pk + 1);
        double p1[3], p2[3], p3[3], p4[3];
        cur->points->GetPoint(sw, p1); cur->points->GetPoint(se, p2);
        cur->points->GetPoint(ne, p3); cur->points->GetPoint(nw, p4);
        double d1x = p1[0]-p3[0], d1y = p1[1]-p3[1], d1z = p1[2]-p3[2];
        double d2x = p2[0]-p4[0], d2y = p2[1]-p4[1], d2z = p2[2]-p4[2];
        int t1[3], t2[3];
        if (d1x*d1x + d1y*d1y + d1z*d1z <= d2x*d2x + d2y*d2y + d2z*d2z) {
            t1[0]=sw; t1[1]=se; t1[2]=ne; t2[0]=sw; t2[1]=ne; t2[2]=nw;
        } else {
            t1[0]=sw; t1[1]=se; t1[2]=nw; t2[0]=se; t2[1]=ne; t2[2]=nw;
        }
        const SurfCell& gc = m_surfGrid[pm][pk];
        unsigned char r, g, bl;
        GetColorFromValue(isAmpMode ? (double)gc.a : (double)gc.t, r, g, bl);
        for (int tr = 0; tr < 2; tr++) {
            const int* tv = (tr == 0) ? t1 : t2;
            vtkIdType tri[3] = {tv[0], tv[1], tv[2]};
            cur->cells->InsertNextCell(3, tri);
            cur->cellColors->InsertNextTuple3(r, g, bl);
            cur->triA.push_back(gc.a);
            cur->triT.push_back(gc.t);
        }
        m_meshPatchDone.insert(pkey);
    };
    auto flushGridRow = [&](int m) {
        if (m < 0) return;
        auto it = m_surfGrid.find(m);
        if (it == m_surfGrid.end()) return;
        for (auto& kv : it->second) createPatch(m, kv.first);
    };

    auto buildRow = [&](long long rkey) {
        int jj = (int)(rkey & 0xFFFFFFFFLL);
        auto itr = m_surfDataRows.find(rkey);
        if (itr == m_surfDataRows.end()) { m_surfRowDone.insert(rkey); return; }
        const std::unordered_map<int, SurfCell>& row = itr->second;
        if (row.empty()) { m_surfDataRows.erase(rkey); m_surfRowDone.insert(rkey); return; }
        m_surfRowDone.insert(rkey);

        // Fill interior k gaps so the patch test has no holes inside the row.
        std::unordered_map<int, SurfCell> rowFilled;
        if (row.size() > 1) {
            std::vector<int> ks;
            ks.reserve(row.size());
            for (auto& kv : row) ks.push_back(kv.first);
            std::sort(ks.begin(), ks.end());
            for (size_t qi = 0; qi + 1 < ks.size(); qi++) {
                int k0 = ks[qi], k1 = ks[qi + 1];
                const SurfCell& a = row.at(k0);
                const SurfCell& b = row.at(k1);
                for (int k = k0; k <= k1; k++) {
                    double tf = (double)(k - k0) / (double)(k1 - k0);
                    SurfCell c;
                    c.x = a.x + (float)((b.x - a.x) * tf);
                    c.y = a.y + (float)((b.y - a.y) * tf);
                    c.z = a.z + (float)((b.z - a.z) * tf);
                    c.a = a.a + (float)((b.a - a.a) * tf);
                    c.t = a.t + (float)((b.t - a.t) * tf);
                    rowFilled[k] = c;
                }
            }
        } else {
            rowFilled = row;
        }

        // 只有“同一道(pass)且 j 连续相邻”的行才能插值连接；
        // 跨道、跳行或数据缺口时断开另起一段，避免生成点云中
        // 不存在的悬空桥接带。
        long long curKey = rkey;
        int curJ = (int)(curKey & 0xFFFFFFFFLL);
        int prevJ = (int)(m_surfPrevKey & 0xFFFFFFFFLL);
        bool adjacent = m_surfHasPrev &&
                        (curKey >> 32) == (m_surfPrevKey >> 32) &&
                        (curJ == prevJ + 1 || curJ == prevJ - 1);

        if (!adjacent) {
            m_surfGrid.clear();
            m_meshVertIdx.clear();
            m_meshPatchDone.clear();
            m_surfGrid[0] = rowFilled;
            m_surfHasPrev = true;
            m_surfDataN = 0;
            qDebug() << "SURF segment break at j=" << curJ
                     << " prevJ=" << (m_surfHasPrev ? prevJ : -1)
                     << " pass=" << (int)(curKey >> 32);
        } else {
            const std::unordered_map<int, SurfCell>& prev = m_surfPrevRow;
            int prevMin = prev.begin()->first, prevMax = prevMin;
            for (auto& kv : prev) {
                if (kv.first < prevMin) prevMin = kv.first;
                if (kv.first > prevMax) prevMax = kv.first;
            }
            int curMin = rowFilled.begin()->first, curMax = curMin;
            for (auto& kv : rowFilled) {
                if (kv.first < curMin) curMin = kv.first;
                if (kv.first > curMax) curMax = kv.first;
            }
            int spanMin = (prevMin < curMin) ? prevMin : curMin;
            int spanMax = (prevMax > curMax) ? prevMax : curMax;
            int m0 = m_surfDataN * 3;
            for (int s = 1; s <= 3; s++) {
                int m = m0 + s;
                if (s == 3) {
                    m_surfGrid[m] = rowFilled;
                } else {
                    double tf = (double)s / 3.0;
                    std::unordered_map<int, SurfCell> sub;
                    for (int k = spanMin; k <= spanMax; k++) {
                        auto itp = prev.find(k);
                        const SurfCell* pp = nullptr;
                        if (itp != prev.end()) pp = &itp->second;
                        else pp = (k < prevMin) ? &prev.at(prevMin) : &prev.at(prevMax);
                        auto itc = rowFilled.find(k);
                        const SurfCell* cc = nullptr;
                        if (itc != rowFilled.end()) cc = &itc->second;
                        else cc = (k < curMin) ? &rowFilled.at(curMin) : &rowFilled.at(curMax);
                        SurfCell c;
                        c.x = pp->x + (float)((cc->x - pp->x) * tf);
                        c.y = pp->y + (float)((cc->y - pp->y) * tf);
                        c.z = pp->z + (float)((cc->z - pp->z) * tf);
                        c.a = pp->a + (float)((cc->a - pp->a) * tf);
                        c.t = pp->t + (float)((cc->t - pp->t) * tf);
                        sub[k] = c;
                    }
                    m_surfGrid[m] = sub;
                }
                flushGridRow(m - 1);
            }
            m_surfDataN++;
        }
        m_surfPrevKey = rkey;
        m_surfPrevRow = std::move(rowFilled);
        m_surfDataRows.erase(rkey);
    };

    for (int i = m_meshBuiltCount; i < n; i++) {
        int kk = m_gridK[i], jj = m_gridJ[i];
        long long key = ((long long)kk << 32) | (unsigned int)(jj & 0xFFFFFFFFLL);
        m_meshIdx[key] = i;
        SurfCell sc;
        sc.x = m_cloudXYZ[i*3+0]; sc.y = m_cloudXYZ[i*3+1]; sc.z = m_cloudXYZ[i*3+2];
        sc.a = (float)savedAmpValues[i]; sc.t = (float)savedTofValues[i];
        long long rkey = ((long long)m_surfPointPass[i] << 32) | (unsigned int)(jj & 0xFFFFFFFFLL);
        m_surfDataRows[rkey][kk] = sc;
        m_surfRowCnt[rkey]++;
        m_surfRowLastSeen[rkey] = i;
    }

    if (forceTail) {
        // Finalize every remaining row of the current band (end of drawing).
        std::vector<long long> tail;
        for (auto& kv : m_surfRowCnt)
            if ((int)(kv.first >> 32) == m_meshCurBand) tail.push_back(kv.first);
        std::sort(tail.begin(), tail.end());
        for (long long rkey : tail) buildRow(rkey);
        m_meshBuiltCount = n;
        if (cur->points) cur->points->Modified();
        if (cur->cells) cur->cells->Modified();
        if (cur->cellColors) cur->cellColors->Modified();
        if (cur->poly) cur->poly->Modified();
        if (cur->mapper) cur->mapper->Modified();
        if (cur->cells->GetNumberOfCells() >= kChunkMaxCells) newSurfaceChunk();
        return;
    }

    // Find where the new band starts inside this batch (per-point pass index).
    int switchI = m_meshBuiltCount;
    while (switchI < n && m_surfPointPass[switchI] == m_meshCurBand) switchI++;

    auto rowReady = [&](long long rkey) -> bool {
        auto itc = m_surfRowCnt.find(rkey);
        if (itc == m_surfRowCnt.end() || itc->second < 47) return false;
        bool full = (itc->second >= 49);
        bool stale = ((n - 1) - m_surfRowLastSeen[rkey]) >= 150;
        return full || stale;
    };

    // Finish the old band's rows inside this batch.
    for (int i = m_meshBuiltCount; i < switchI; i++) {
        int jj = m_gridJ[i];
        long long rkey = ((long long)m_meshCurBand << 32) | (unsigned int)(jj & 0xFFFFFFFFLL);
        if (!rowReady(rkey)) continue;
        buildRow(rkey);
    }

    if (switchI < n) {
        // Band switch: finish the previous band's remaining rows, then
        // reset the per-band state before processing the new band's points.
        std::vector<long long> oldRows;
        for (auto& kv : m_surfRowCnt)
            if ((int)(kv.first >> 32) == m_meshCurBand) oldRows.push_back(kv.first);
        std::sort(oldRows.begin(), oldRows.end());
        for (long long rkey : oldRows) buildRow(rkey);
        m_surfGrid.clear();
        m_meshVertIdx.clear();
        m_meshPatchDone.clear();
        m_surfHasPrev = false;
        m_surfPrevKey = -1;
        m_surfDataN = 0;
        m_meshCurBand = m_surfPointPass[switchI];
        qDebug() << "SURF band switch to" << m_meshCurBand;
    }

    // Build the new band's completed rows.
    for (int i = switchI; i < n; i++) {
        int jj = m_gridJ[i];
        long long rkey = ((long long)m_surfPointPass[i] << 32) | (unsigned int)(jj & 0xFFFFFFFFLL);
        if (!rowReady(rkey)) continue;
        buildRow(rkey);
    }
    m_meshBuiltCount = n;
    if (cur->points) cur->points->Modified();
    if (cur->cells) cur->cells->Modified();
    if (cur->cellColors) cur->cellColors->Modified();
    if (cur->poly) cur->poly->Modified();
    if (cur->mapper) cur->mapper->Modified();
    if (cur->cells->GetNumberOfCells() >= kChunkMaxCells) newSurfaceChunk();
}

void MainWindow7::recolorSurfaceMesh()
{
    for (auto& ch : m_surfChunks) {
        if (!ch.cellColors) continue;
        for (size_t q = 0; q < ch.triA.size(); q++) {
            unsigned char r, g, bl;
            GetColorFromValue(isAmpMode ? (double)ch.triA[q] : (double)ch.triT[q], r, g, bl);
            ch.cellColors->SetTuple3((vtkIdType)q, r, g, bl);
        }
        if (ch.cellColors) ch.cellColors->Modified();
        if (ch.poly) ch.poly->Modified();
        if (ch.mapper) ch.mapper->Modified();
    }
}

