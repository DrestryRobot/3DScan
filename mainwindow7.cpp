#include "mainwindow7.h"
#include "cuda_runtime_api.h"
#include "ui_mainwindow7.h"

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

// 最大点数限制
static vtkIdType currentIndex = 0;

MainWindow7::MainWindow7(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow7)
    , m_scan(nullptr)
    , m_isScanning(false)
    , cudaProcessor(nullptr)
{
    ui->setupUi(this);

    initWidget(); // 初始化界面

    initSlot();   // 初始化信号

    initVTK();    // 初始化VTK

    // 初始化 Scan 对象
    m_scan = new Scan(this);

    // 延迟注册 VBO (等待 OpenGL 上下文就绪)
    QTimer::singleShot(200, this, &MainWindow7::registerVBOWithCUDA);
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

    return QMainWindow::eventFilter(obj, event);
}

//初始化参数
void MainWindow7::initParam()
{

}

// 初始化界面
void MainWindow7::initWidget()
{
    // 自适应窗口
    this->centralWidget()->setLayout(ui->gridLayout_2);

    // 安装事件过滤器到colorBarWidget
    ui->colorBarWidget->installEventFilter(this);

}

// 初始化信号
void MainWindow7::initSlot()
{

}

// 初始化VTK
void MainWindow7::initVTK()
{
    // 创建渲染器
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0, 0, 0);

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

    renderer->ResetCamera();

    renderWindow->Render();
}

// // 初始化点云
// void MainWindow7::initPointCloud()
// {
//     savedAmpValues.clear();
//     savedTofValues.clear();

//     points = vtkSmartPointer<vtkPoints>::New();
//     vertices = vtkSmartPointer<vtkCellArray>::New();
//     colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
//     colors->SetNumberOfComponents(3);
//     colors->SetName("Colors");

//     polyData = vtkSmartPointer<vtkPolyData>::New();
//     polyData->SetPoints(points);
//     polyData->SetVerts(vertices);
//     polyData->GetPointData()->SetScalars(colors);

//     mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//     mapper->SetInputData(polyData);

//     actor = vtkSmartPointer<vtkActor>::New();
//     actor->SetMapper(mapper);
//     actor->GetProperty()->SetPointSize(3);
//     renderer->AddActor(actor);

//     renderWindow->Render();
// }

// void MainWindow7::AddPointCloud(const double pose[], const double amp[], const double tof[], double si, int beam)
// {
//     int centerBeam = (beam - 1) / 2;
//     double spacing = 0.3;

//     vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
//     transform->PostMultiply();
//     transform->RotateZ(-pose[3]);
//     transform->RotateY(pose[4]);
//     transform->RotateX(pose[5]);

//     for (int i = 0; i < beam; i++) {
//         if (amp[i] == 0.0 || tof[i] == 0.0) continue;

//         double offset = (i - centerBeam) * spacing;
//         double localPoint[3] = {0.0, offset, -si * 0.5};
//         double globalPoint[3];
//         transform->TransformPoint(localPoint, globalPoint);

//         double worldX = globalPoint[0] + pose[0];
//         double worldY = globalPoint[1] + pose[1];
//         double worldZ = globalPoint[2] + pose[2];

//         // 保存原始数据
//         savedAmpValues.push_back(amp[i]);
//         savedTofValues.push_back(tof[i]);

//         // 获取当前值
//         double value = isAmpMode ? amp[i] : tof[i];
//         unsigned char r, g, b;
//         GetColorFromValue(value, r, g, b);

//         // 追加点
//         // vtkIdType pid = points->InsertNextPoint(worldX, worldY, worldZ);
//         // vertices->InsertNextCell(1, &pid);
//         // colors->InsertNextTuple3(r, g, b);

//         // 环形缓冲
//         vtkIdType id = (currentIndex % MAX_POINTS);
//         if (points->GetNumberOfPoints() < MAX_POINTS) {
//             // 初始阶段：点数未满，正常插入
//             vtkIdType pid = points->InsertNextPoint(worldX, worldY, worldZ);
//             vertices->InsertNextCell(1, &pid);
//             colors->InsertNextTuple3(r, g, b);
//         } else {
//             // 点数已满：覆盖旧点
//             points->SetPoint(id, worldX, worldY, worldZ);
//             colors->SetTuple3(id, r, g, b);
//         }
//         currentIndex++;
//     }

//     if(currentIndex % 100 == 0)
//     {
//         points->Modified();
//         colors->Modified();
//         polyData->Modified();

//         renderer->ResetCamera();

//         renderWindow->Render();
//     }
// }

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

    // 创建自定义Actor
    vboActor = vtkSmartPointer<vtkVBOActor>::New();
    vboActor->SetMapper(mapper);
    vboActor->VisibilityOn();
    renderer->AddActor(vboActor);

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
        cudaProcessor = createCUDAProcessor(MAX_POINTS_PER_FRAME);
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
    } else {
        qDebug() << "Failed to register VBO to CUDA";
        delete vboPoints;
        vboPoints = nullptr;
        delete vboColors;
        vboColors = nullptr;
    }
}

void MainWindow7::AddPointCloud(const double pose[], const double amp[],
                                const double tof[], double si, int beam)
{
    if (beam <= 0) return;
    if (!isCUDAAvailable()) return;

    if (!cudaProcessor) {
        cudaProcessor = createCUDAProcessor(beam);
        if (!cudaProcessor) return;
    }

    int result = processDirectVBO(cudaProcessor, pose, amp, tof, si, beam,
                                  isAmpMode ? 1 : 0);

    if (result == 0) {
        for (int i = 0; i < beam; i++) {
            if (amp[i] != 0.0 && tof[i] != 0.0) {
                savedAmpValues.push_back(amp[i]);
                savedTofValues.push_back(tof[i]);
            }
        }

        if (beam > vboMaxPoints) {
            qDebug() << "Beam exceeds VBO capacity:" << beam << ">" << vboMaxPoints;
            return;
        }

        // 直接告诉 vtkVBOActor 使用的 VBO
        vboActor->SetVBOIDs(vboPoints->bufferId(), vboColors->bufferId());
        vboActor->SetValidPointCount(beam);
        vboActor->SetVBOInitialized(true);

        // 确保 CUDA 已经完成写入
        cudaDeviceSynchronize();

        currentIndex += beam;

        // 只在第一次或相机重置时重置相机
        if (currentIndex == beam) {  // 第一次添加点
            renderer->ResetCamera();
        }

        renderWindow->Render();

    } else {
        qDebug() << "VBO Direct failed:" << result;
    }
}

void MainWindow7::renderFrame(const ScanFrame& frame)
{
    // 添加点云
    AddPointCloud(frame.pose, frame.amp, frame.tof, frame.si, frame.beam);

    static int renderCount = 0;
    static QElapsedTimer renderTimer;
    if (!renderTimer.isValid()) {
        renderTimer.start();
    }

    renderCount++;
    if (renderTimer.elapsed() >= 1000) {
        double renderFps = renderCount;
        qDebug() << "渲染帧率:" << renderFps << "fps";
        renderCount = 0;
        renderTimer.restart();


        // 打印实时数据到 UI
        QString text = QString("Frame: %1 SI: %2 Beam: %3 "
                               "Pose: %4 %5 %6 %7 %8 %9 "
                               "AMP[0]: %10 TOF[0]: %11")
                           .arg(frame.ipoc)
                           .arg(frame.si, 0, 'f', 2)   // 两位小数
                           .arg(frame.beam)
                           .arg(frame.pose[0], 0, 'f', 2)
                           .arg(frame.pose[1], 0, 'f', 2)
                           .arg(frame.pose[2], 0, 'f', 2)
                           .arg(frame.pose[3], 0, 'f', 2)
                           .arg(frame.pose[4], 0, 'f', 2)
                           .arg(frame.pose[5], 0, 'f', 2)
                           .arg(frame.amp[0], 0, 'f', 2)
                           .arg(frame.tof[0], 0, 'f', 2);

        ui->lineEdit->setText(text);
        ui->lineEdit->setCursorPosition(0);
    }
}

void MainWindow7::onMouseClick(vtkObject* obj, unsigned long event, void* clientData, void* callData)
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
    // 初始化点云
    initPointCloud();

    connect(m_scan, &Scan::newFrameAvailable, this, &MainWindow7::renderFrame);

    // 启动扫描
    if (!m_isScanning) {
        m_scan->start();
        m_isScanning = true;
        ui->pushButton_7->setText("停止扫描");
        qDebug() << "Scan started for rendering";
    }
}

// 停止绘制
void MainWindow7::on_pushButton_2_clicked()
{

}

// 结束绘制
void MainWindow7::on_pushButton_3_clicked()
{

}

// 保存数据
void MainWindow7::on_pushButton_5_clicked()
{

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
    int count = 0;
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

    std::string line;

    std::vector<std::string> cache;

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

        cache.push_back(line);

        if (cache.size() < 3) continue;

        std::vector<std::string> cells_main = parseCSVLine(cache[0]);

        std::vector<std::string> cells_current = parseCSVLine(line);

        // 读取波束
        if(BEAM != -1 && safe_stod(cells_main[BEAM]))
        {
            beam = safe_stod(cells_main[BEAM]);
        }
        if(BEAM == -1) beam = 49;

        // 读取龙门坐标
        if((LX != -1 || LY != -1) && (safe_stod(cells_main[LX]) || safe_stod(cells_main[LY])))
        {
            longmen[0] = safe_stod(cells_main[LX]);
            longmen[1] = safe_stod(cells_main[LY]);
        }
        if(LX == -1) longmen[0] = 0.0;
        if(LY == -1) longmen[1] = 0.0;

        // 读取机器人位姿
        double pose[6];
        for (int i = 0; i < 6; i++)
        {
            pose[i] = safe_stod(cells_main[X + i]);
        }
        pose[0] = pose[0] + longmen[1];
        pose[1] = pose[1] - longmen[0];

        // 读取SI值
        if(SI != -1) si = safe_stod(cells_current[SI]);
        if(SI == -1) si = 0.0;

        // 读取AMP值
        for (int i = 0; i < beam; i++)
        {
            int ampIndex = AMP_1 + 2 * i;
            if (ampIndex < (int)cells_current.size()) {
                amp[i] = safe_stod(cells_current[ampIndex]);
            } else {
                amp[i] = 0.0;
            }
        }

        // 读取TOF值
        for (int i = 0; i < beam; i++)
        {
            int tofIndex = TOF_1 + 2 * i;
            if (tofIndex < (int)cells_current.size()) {
                tof[i] = safe_stod(cells_current[tofIndex]);
            } else {
                tof[i] = 0.0;
            }
        }

        // 添加点云
        AddPointCloud(pose, amp, tof, si, beam);

        // 移除已处理的最早行
        cache.erase(cache.begin());
    }

    file.close();

    if (canceled) {
        progressDialog.setValue(100);
        return;
    }

    progressDialog.setValue(100);
}

// 重置数据
void MainWindow7::on_pushButton_4_clicked()
{
    initVTK();
}

// 数据模式
void MainWindow7::on_pushButton_8_clicked()
{
    if (isAmpMode)
    {
        ui->pushButton_8->setText("TOF模式");

        if(isCloudMode && !savedTofValues.empty())
        {
            for (size_t i = 0; i < savedTofValues.size(); i++) {
                unsigned char r, g, b;
                GetColorFromValue(savedTofValues[i], r, g, b);
                colors->SetTuple3(i, r, g, b);
            }

            colors->Modified();

            polyData->GetPointData()->SetScalars(colors);
        }
    }
    else
    {
        ui->pushButton_8->setText("AMP模式");

        if(isCloudMode && !savedAmpValues.empty())
        {
            for (size_t i = 0; i < savedAmpValues.size(); i++) {
                unsigned char r, g, b;
                GetColorFromValue(savedAmpValues[i], r, g, b);
                colors->SetTuple3(i, r, g, b);
            }

            colors->Modified();

            polyData->GetPointData()->SetScalars(colors);
        }
    }

    renderWindow->Render();

    isAmpMode = !isAmpMode;
}

// 数据测量
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
        m_scan->start();
        m_isScanning = true;
        ui->pushButton_7->setText("停止扫描");

        qDebug() << "Scan started";

    } else {
        // ===== 停止扫描 =====
        m_scan->stop();
        m_isScanning = false;
        ui->pushButton_7->setText("开始扫描");

        qDebug() << "Scan stopped";
    }
}




