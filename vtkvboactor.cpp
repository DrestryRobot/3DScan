#include "vtkVBOActor.h"
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkMatrix4x4.h>
#include <QOpenGLContext>
#include <QDebug>

vtkStandardNewMacro(vtkVBOActor);

vtkVBOActor::vtkVBOActor()
{
    this->SetUseBounds(false);
}

vtkVBOActor::~vtkVBOActor()
{
    if (gl) {
        if (shaderProgram) {
            gl->glDeleteProgram(shaderProgram);
        }
        if (vao) {
            gl->glDeleteVertexArrays(1, &vao);
        }
    }
}

bool vtkVBOActor::initGL()
{
    if (glInitialized) return true;

    QOpenGLContext* ctx = QOpenGLContext::currentContext();
    if (!ctx) {
        qDebug() << "No OpenGL context";
        return false;
    }

    gl = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_3_Core>(ctx);
    if (!gl) {
        qDebug() << "Failed to get OpenGL 3.3 functions";
        return false;
    }

    glInitialized = true;
    qDebug() << "OpenGL 3.3 initialized for VBO Actor";
    return true;
}

bool vtkVBOActor::initShaderAndVAO()
{
    if (shaderInitialized) return true;

    if (!initGL()) return false;

    // ===== 顶点着色器 =====
    const char* vsSource =
        "#version 330 core\n"
        "layout(location = 0) in vec3 position;\n"
        "layout(location = 1) in vec3 color;\n"
        "uniform mat4 viewMatrix;\n"
        "uniform mat4 projMatrix;\n"
        "out vec3 fragColor;\n"
        "void main() {\n"
        "    gl_PointSize = 3.0;\n"
        "    gl_Position = projMatrix * viewMatrix * vec4(position, 1.0);\n"
        "    fragColor = color;\n"
        "}\n";

    // ===== 片段着色器 =====
    const char* fsSource =
        "#version 330 core\n"
        "in vec3 fragColor;\n"
        "out vec4 outputColor;\n"
        "void main() {\n"
        "    outputColor = vec4(fragColor, 1.0);\n"
        "}\n";

    // 编译顶点着色器
    unsigned int vs = gl->glCreateShader(GL_VERTEX_SHADER);
    gl->glShaderSource(vs, 1, &vsSource, nullptr);
    gl->glCompileShader(vs);

    int success;
    gl->glGetShaderiv(vs, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        gl->glGetShaderInfoLog(vs, 512, nullptr, log);
        qDebug() << "VS compile error:" << log;
        return false;
    }

    // 编译片段着色器
    unsigned int fs = gl->glCreateShader(GL_FRAGMENT_SHADER);
    gl->glShaderSource(fs, 1, &fsSource, nullptr);
    gl->glCompileShader(fs);

    gl->glGetShaderiv(fs, GL_COMPILE_STATUS, &success);
    if (!success) {
        char log[512];
        gl->glGetShaderInfoLog(fs, 512, nullptr, log);
        qDebug() << "FS compile error:" << log;
        gl->glDeleteShader(vs);
        return false;
    }

    // 链接着色器
    shaderProgram = gl->glCreateProgram();
    gl->glAttachShader(shaderProgram, vs);
    gl->glAttachShader(shaderProgram, fs);
    gl->glLinkProgram(shaderProgram);

    gl->glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char log[512];
        gl->glGetProgramInfoLog(shaderProgram, 512, nullptr, log);
        qDebug() << "Link error:" << log;
        gl->glDeleteShader(vs);
        gl->glDeleteShader(fs);
        gl->glDeleteProgram(shaderProgram);
        shaderProgram = 0;
        return false;
    }

    gl->glDeleteShader(vs);
    gl->glDeleteShader(fs);

    // ===== 创建VAO =====
    gl->glGenVertexArrays(1, &vao);
    gl->glBindVertexArray(vao);

    // 位置属性
    gl->glBindBuffer(GL_ARRAY_BUFFER, vboPointsID);
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);

    // 颜色属性
    gl->glBindBuffer(GL_ARRAY_BUFFER, vboColorsID);
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(1, 3, GL_UNSIGNED_BYTE, GL_TRUE, 3 * sizeof(unsigned char), nullptr);

    gl->glBindVertexArray(0);
    gl->glBindBuffer(GL_ARRAY_BUFFER, 0);

    shaderInitialized = true;
    qDebug() << "Shader and VAO initialized";
    return true;
}

int vtkVBOActor::RenderOpaqueGeometry(vtkViewport* viewport)
{
    qDebug() << "[VBOActor] >>>>>>> RENDER OPAQUE ENTRY <<<<<<<";

    // qDebug() << "[VBOActor] RenderOpaqueGeometry called";

    vtkRenderer* renderer = vtkRenderer::SafeDownCast(viewport);
    if (!renderer) return 0;

    // 调用你原来的 Render()
    this->Render(renderer, nullptr);
    return 1;
}

void vtkVBOActor::Render(vtkRenderer* renderer, vtkMapper* mapper)
{
    qDebug() << "[VBOActor] >>>>>>> RENDER() CALLED <<<<<<<";

    if (!vboInitialized || validPointCount <= 0 ||
        vboPointsID == 0 || vboColorsID == 0) {
        return;
    }

    if (!vao || !shaderProgram) {
        qDebug() << "VAO or shader not initialized!";
        return;
    }

    vtkOpenGLRenderWindow* renWin =
        vtkOpenGLRenderWindow::SafeDownCast(renderer->GetRenderWindow());
    if (!renWin) return;

    renWin->MakeCurrent();

    if (!initGL()) return;
    if (!shaderInitialized) {
        if (!initShaderAndVAO()) return;
    }

    // ===== 获取相机矩阵 =====
    vtkCamera* camera = renderer->GetActiveCamera();
    if (!camera) return;

    vtkMatrix4x4* viewMat = camera->GetViewTransformMatrix();
    vtkMatrix4x4* projMat = camera->GetProjectionTransformMatrix(
        renderer->GetTiledAspectRatio(), 0, 1);

    // 转换为 OpenGL 列主序
    float view[16], proj[16];
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            view[j * 4 + i] = viewMat->GetElement(i, j);
            proj[j * 4 + i] = projMat->GetElement(i, j);
        }
    }

    // ===== 设置视口 =====
    int* size = renderer->GetSize();
    gl->glViewport(0, 0, size[0], size[1]);
    gl->glEnable(GL_DEPTH_TEST);

    // ===== 使用着色器 =====
    gl->glUseProgram(shaderProgram);

    GLint viewLoc = gl->glGetUniformLocation(shaderProgram, "viewMatrix");
    GLint projLoc = gl->glGetUniformLocation(shaderProgram, "projMatrix");
    gl->glUniformMatrix4fv(viewLoc, 1, GL_FALSE, view);
    gl->glUniformMatrix4fv(projLoc, 1, GL_FALSE, proj);

    // ===== 绘制 =====
    gl->glBindVertexArray(vao);
    gl->glDrawArrays(GL_POINTS, 0, validPointCount);
    gl->glBindVertexArray(0);

    gl->glUseProgram(0);

    static int frameCount = 0;
    if (++frameCount % 100 == 0) {
        qDebug() << "VBO render:" << validPointCount << "points";
    }
}