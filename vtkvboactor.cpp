#include "vtkVBOActor.h"
#include <vtkRenderer.h>
#include <vtkCamera.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkMatrix4x4.h>
#include <vtkOpenGLFramebufferObject.h>
#include <QOpenGLContext>
#include <QDebug>
#include <vector>

vtkStandardNewMacro(vtkVBOActor);

vtkVBOActor::vtkVBOActor()
{
    // Bounds are provided by SetCloudBounds()/GetBounds(), so the actor is
    // included in camera framing and clipping-range calculations.
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
        if (m_indexVBO) {
            gl->glDeleteBuffers(1, &m_indexVBO);
        }
    }
}

void vtkVBOActor::SetCloudBounds(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
    cloudBounds[0] = xmin; cloudBounds[1] = xmax;
    cloudBounds[2] = ymin; cloudBounds[3] = ymax;
    cloudBounds[4] = zmin; cloudBounds[5] = zmax;
    cloudBoundsSet = true;
}

double* vtkVBOActor::GetBounds()
{
    return cloudBoundsSet ? cloudBounds : this->Superclass::GetBounds();
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
        "uniform mat4 mvpMatrix;\n"
        "uniform int u_stride;\n"
        "out vec3 fragColor;\n"
        "void main() {\n"
        "    gl_PointSize = 2.0;\n"
        "    if (u_stride > 1 && (gl_VertexID % u_stride) != 0) {\n"
        "        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);\n"
        "        fragColor = color;\n"
        "        return;\n"
        "    }\n"
        "    gl_Position = mvpMatrix * vec4(position, 1.0);\n"
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

    // 缓存 uniform 位置（每帧查询很慢）
    mvpLoc = gl->glGetUniformLocation(shaderProgram, "mvpMatrix");
    strideLoc = gl->glGetUniformLocation(shaderProgram, "u_stride");

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
    static bool entryLogged = false;
    if (!entryLogged) {
        qDebug() << "[VBOActor] RenderOpaqueGeometry called, valid="
                 << validPointCount << "vboInit=" << vboInitialized
                 << "ids=" << vboPointsID << vboColorsID;
        entryLogged = true;
    }

    vtkRenderer* renderer = vtkRenderer::SafeDownCast(viewport);
    if (!renderer) return 0;

    // 调用你原来的 Render()
    this->Render(renderer, nullptr);
    return 1;
}

void vtkVBOActor::Render(vtkRenderer* renderer, vtkMapper* mapper)
{

    if (!vboInitialized || validPointCount <= 0 ||
        vboPointsID == 0 || vboColorsID == 0) {
        return;
    }

    vtkOpenGLRenderWindow* renWin =
        vtkOpenGLRenderWindow::SafeDownCast(renderer->GetRenderWindow());
    if (!renWin) return;

    renWin->MakeCurrent();

    if (!initGL()) return;

    // VTK renders into its render framebuffer object; MakeCurrent() may have
    // reset the framebuffer binding, so re-bind it before drawing.
    vtkOpenGLFramebufferObject* fbo = renWin->GetRenderFramebuffer();
    if (fbo) {
        fbo->Bind();
    }
    if (!shaderInitialized) {
        if (!initShaderAndVAO()) return;
    }

    if (!vao || !shaderProgram) {
        qDebug() << "VAO or shader not initialized!";
        return;
    }

    // ===== 获取相机矩阵（MVP 缓存：相机/视口不变时复用） =====
    vtkCamera* camera = renderer->GetActiveCamera();
    if (!camera) return;

    int* size = renderer->GetSize();
    if (!m_mvpValid || camera->GetMTime() != m_camMTime ||
        renderer->GetMTime() != m_renMTime ||
        size[0] != m_vpW || size[1] != m_vpH) {
        // Replicate vtkOpenGLCamera::GetKeyMatrices exactly:
        //   WCVC = transpose(model-view)
        //   VCDC = transpose(projection)
        //   WCDC = WCVC * VCDC
        vtkMatrix4x4* projMat = camera->GetProjectionTransformMatrix(
            renderer->GetTiledAspectRatio(), -1, 1);
        vtkMatrix4x4* viewMat = camera->GetModelViewTransformMatrix();

        vtkMatrix4x4* wcvc = vtkMatrix4x4::New();
        wcvc->DeepCopy(viewMat);
        wcvc->Transpose();
        vtkMatrix4x4* vcdc = vtkMatrix4x4::New();
        vcdc->DeepCopy(projMat);
        vcdc->Transpose();
        vtkMatrix4x4* wcdc = vtkMatrix4x4::New();
        vtkMatrix4x4::Multiply4x4(wcvc, vcdc, wcdc);
        for (int i = 0; i < 16; i++) {
            m_cachedMvp[i] = static_cast<float>(wcdc->Element[0][i]);
        }
        wcvc->Delete();
        vcdc->Delete();
        wcdc->Delete();
        m_camMTime = camera->GetMTime();
        m_renMTime = renderer->GetMTime();
        m_vpW = size[0];
        m_vpH = size[1];
        m_mvpValid = true;
    }

    // ===== 设置视口 =====
    gl->glViewport(0, 0, size[0], size[1]);
    gl->glEnable(GL_DEPTH_TEST);
    // 点云表面点深度几乎相等，GL_LESS 会让相邻点在旋转时随机通过/遮挡
    // （点阵忽明忽暗）；LEQUAL 让等深度点稳定通过，消除旋转时的亮度抖动。
    gl->glDepthFunc(GL_LEQUAL);
    gl->glEnable(GL_PROGRAM_POINT_SIZE);

    // ===== 使用着色器 =====
    gl->glUseProgram(shaderProgram);

    gl->glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, m_cachedMvp);
    // 抽稀改由索引绘制完成（只提交 1/stride 的顶点），着色器不再丢弃顶点
    gl->glUniform1i(strideLoc, 1);

    // ===== 绘制 =====
    static bool firstDrawLogged = false;
    if (!firstDrawLogged) {
        qDebug() << "VBOActor first draw, points:" << validPointCount;
        firstDrawLogged = true;
    }

    gl->glBindVertexArray(vao);
    if (displayStride > 1) {
        // 索引绘制：只绘制下标为 stride 倍数的点
        int count = (validPointCount + displayStride - 1) / displayStride;
        if (m_indexStride != displayStride || count > m_indexCapacity) {
            if (!m_indexVBO) gl->glGenBuffers(1, &m_indexVBO);
            int cap = (m_indexCapacity < 1024) ? 1024 : m_indexCapacity;
            while (cap < count) cap *= 2;
            std::vector<GLuint> idx((size_t)cap);
            for (int i = 0; i < cap; i++) idx[(size_t)i] = (GLuint)(i * displayStride);
            gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexVBO);
            gl->glBufferData(GL_ELEMENT_ARRAY_BUFFER, (GLsizeiptr)cap * sizeof(GLuint),
                             idx.data(), GL_STATIC_DRAW);
            m_indexStride = displayStride;
            m_indexCapacity = cap;
        }
        m_indexCount = count;
        gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexVBO);
        gl->glDrawElements(GL_POINTS, m_indexCount, GL_UNSIGNED_INT, nullptr);
        gl->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    } else {
        gl->glDrawArrays(GL_POINTS, 0, validPointCount);
    }
    GLenum err = gl->glGetError();
    if (err != GL_NO_ERROR) {
        qDebug() << "GL error after draw:" << err;
    }
    gl->glBindVertexArray(0);

    gl->glUseProgram(0);

    static int frameCount = 0;
    if (++frameCount % 100 == 0) {
        qDebug() << "VBO render:" << validPointCount << "points";
    }
}
