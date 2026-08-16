#ifndef VTK_VBO_ACTOR_H
#define VTK_VBO_ACTOR_H

#include <vtkActor.h>
#include <vtkObjectFactory.h>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLVersionFunctionsFactory>
#include <QOpenGLFunctions>

// CUDA接口
#include "algorithm.h"

class vtkVBOActor : public vtkActor
{
public:
    static vtkVBOActor* New();
    vtkTypeMacro(vtkVBOActor, vtkActor);

    void SetCUDAProcessor(CUDAProcessorHandle handle) { cudaHandle = handle; }
    void SetVBOInitialized(bool initialized) { vboInitialized = initialized; }
    void SetValidPointCount(int count) { validPointCount = count; }
    // 显示抽稀步长：只绘制下标为 stride 倍数的点，用于大点云下保持渲染帧率
    void SetDisplayStride(int stride)
    {
        displayStride = (stride > 1) ? stride : 1;
    }
    void SetVBOIDs(unsigned int pointsID, unsigned int colorsID)
    {
        vboPointsID = pointsID;
        vboColorsID = colorsID;
    }

    void Render(vtkRenderer* renderer, vtkMapper* mapper) override;
    void SetCloudBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax);
    double* GetBounds() override;
    int RenderOpaqueGeometry(vtkViewport* viewport) override;

protected:
    vtkVBOActor();
    ~vtkVBOActor();

private:
    CUDAProcessorHandle cudaHandle = nullptr;
    double cloudBounds[6] = {0, 0, 0, 0, 0, 0};
    bool cloudBoundsSet = false;
    bool vboInitialized = false;
    int validPointCount = 0;
    int displayStride = 1;

    unsigned int vboPointsID = 0;
    unsigned int vboColorsID = 0;

    // OpenGL 3.3 核心函数
    QOpenGLFunctions_3_3_Core* gl = nullptr;
    bool glInitialized = false;

    // 着色器
    unsigned int shaderProgram = 0;
    bool shaderInitialized = false;
    // 缓存 uniform 位置（避免每帧 glGetUniformLocation 查询）
    GLint mvpLoc = -1;
    GLint strideLoc = -1;
    // MVP 缓存：相机/窗口尺寸不变时复用，避免每帧 vtkMatrix4x4 分配与矩阵运算
    float m_cachedMvp[16];
    bool m_mvpValid = false;
    unsigned long m_camMTime = 0;
    unsigned long m_renMTime = 0;
    int m_vpW = -1;
    int m_vpH = -1;

    // VAO
    unsigned int vao = 0;
    // 索引绘制（显示抽稀时只提交 1/stride 的顶点，GPU 不再全量处理）
    unsigned int m_indexVBO = 0;
    int m_indexCapacity = 0;     // 索引缓冲容量（按 2 的幂增长，避免每帧重建）
    int m_indexStride = -1;      // 上次构建索引时的 stride
    int m_indexCount = 0;        // 本次实际绘制的索引数

    bool initGL();
    bool initShaderAndVAO();
};

#endif
