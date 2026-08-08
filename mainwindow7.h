#ifndef MAINWINDOW7_H
#define MAINWINDOW7_H

#include <QMainWindow>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCamera.h>
#include <vtkLightKit.h>

#include <vtkPSphereSource.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkTextProperty.h>

#include <vtkTransform.h>
#include <vtkGlyph3D.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>

#include <QFileDialog>
#include <QMessageBox>

#include <QString>
#include <fstream>
#include <sstream>

#include <vtkPlaneSource.h>

#include <vtkPointPicker.h>
#include <vtkCallbackCommand.h>

#include <vtkPNGWriter.h>
#include <vtkWindowToImageFilter.h>

#include <vtkKdTree.h>
#include <vtkPropPicker.h>

#include <QProgressDialog>
#include <QPainter>

#include <vtkOpenGLPolyDataMapper.h>

#include "scan.h"
#include <QThread>

#include <vtkPointGaussianMapper.h>
#include <vtkAppendPolyData.h>

#include "algorithm.h"
#include "vtkVBOActor.h"
#include <QOpenGLBuffer>
#include <cuda_runtime.h>
#include <vtkRendererCollection.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow7;
}
QT_END_NAMESPACE

class MainWindow7 : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow7(QWidget *parent = nullptr);

    ~MainWindow7();

protected:
    bool eventFilter(QObject *obj, QEvent *event);

private slots:
    void on_pushButton_clicked();    // 开始绘制

    void on_pushButton_2_clicked();  // 停止绘制

    void on_pushButton_3_clicked();  // 结束绘制

    void on_pushButton_5_clicked();  // 保存数据

    void on_pushButton_6_clicked();  // 加载数据

    void on_pushButton_4_clicked();  // 重置数据

    void on_pushButton_8_clicked();  // 数据模式

    void on_pushButton_10_clicked(); // 数据测量

    void on_pushButton_9_clicked();  // 对齐视点

    void on_pushButton_11_clicked(); // 窗口截图

    void on_pushButton_7_clicked(); // 开始扫描

private:
    Ui::MainWindow7 *ui;

    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;
    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style;
    vtkSmartPointer<vtkCamera> camera;
    vtkSmartPointer<vtkLightKit> lightKit;
    vtkSmartPointer<vtkAxesActor> axesActor;
    vtkSmartPointer<vtkPoints> points;
    vtkSmartPointer<vtkCellArray> vertices;
    vtkSmartPointer<vtkPolyData> polyData;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    // vtkSmartPointer<vtkPointGaussianMapper> mapper;
    // vtkSmartPointer<vtkOpenGLPolyDataMapper> mapper;
    vtkSmartPointer<vtkAppendPolyData> appendFilter;
    // vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkVBOActor> vboActor;
    vtkSmartPointer<vtkUnsignedCharArray> colors;
    vtkSmartPointer<vtkActor> point1Actor;
    vtkSmartPointer<vtkActor> point2Actor;


    // 显示模式
    bool isCloudMode = true;

    // 数据模式
    bool isAmpMode = true;
    std::vector<double> savedAmpValues;
    std::vector<double> savedTofValues;

    // 空间测量
    bool isMeasuring = false;
    double lastPoint[3];
    bool hasLastPoint = false;

    // 扫描相关
    Scan* m_scan;
    QThread m_scanThread;
    // 状态
    bool m_isScanning;
    int m_totalFrames;

    static const int MAX_POINTS = 10000000; // 最大支持100000
    // CUDA处理器
    CUDAProcessorHandle cudaProcessor = nullptr;
    int frameCount = 0;
    static const int MAX_POINTS_PER_FRAME = 10000000;  // 添加这行

    bool vboRegistered = false;

    // ============ 自定义 VBO (OpenGL) ============
    QOpenGLBuffer* vboPoints = nullptr;   // 顶点位置 VBO
    QOpenGLBuffer* vboColors = nullptr;   // 顶点颜色 VBO
    int vboMaxPoints = 0;
    bool vboInitialized = false;

    // 当前帧有效点数
    int currentValidPoints = 0;


    void initParam();      //初始化参数
    void initWidget();     // 初始化界面
    void initSlot();       // 初始化信号
    void initVTK();        // 初始化VTK
    void initPointCloud(); // 初始化点云
    void AddPointCloud(const double pose[], const double amp[], const double tof[], double si, int beam);
                       // 添加点云
    static void onMouseClick(vtkObject *obj, unsigned long event, void *clientData, void *callData); // 鼠标点击
    void pickPoint(int x, int y);   // 测量点
    void GetColorFromValue(double value, unsigned char &r, unsigned char &g, unsigned char &b);
    void renderFrame(const ScanFrame &frame);
    vtkSmartPointer<vtkPolyData> createFramePolyData(const double pose[], const double amp[], const double tof[], double si, int beam);
    void registerVBOWithCUDA();
    void setupCustomVBO();
};

#endif // MAINWINDOW7_H
