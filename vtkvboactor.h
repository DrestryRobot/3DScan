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
    void SetVBOIDs(unsigned int pointsID, unsigned int colorsID)
    {
        vboPointsID = pointsID;
        vboColorsID = colorsID;
    }

    void Render(vtkRenderer* renderer, vtkMapper* mapper) override;
    int RenderOpaqueGeometry(vtkViewport* viewport) override;

protected:
    vtkVBOActor();
    ~vtkVBOActor();

private:
    CUDAProcessorHandle cudaHandle = nullptr;
    bool vboInitialized = false;
    int validPointCount = 0;

    unsigned int vboPointsID = 0;
    unsigned int vboColorsID = 0;

    // OpenGL 3.3 核心函数
    QOpenGLFunctions_3_3_Core* gl = nullptr;
    bool glInitialized = false;

    // 着色器
    unsigned int shaderProgram = 0;
    bool shaderInitialized = false;

    // VAO
    unsigned int vao = 0;

    bool initGL();
    bool initShaderAndVAO();
};

#endif