QT       += core gui widgets opengl openglwidgets network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += force_debug_info
msvc: QMAKE_CXXFLAGS += /MP

# Force qmake to emit absolute paths in Makefiles. Fixes
# "dependent '..\..\..\..\..\Qt\...qspinbox.h' does not exist"
# when the build directory is too deep for relative paths.
QMAKE_PROJECT_DEPTH = 0


# # ============ OpenGL 头文件路径 ============
# # 添加 Windows SDK 的 OpenGL 路径
# INCLUDEPATH += "C:/Program Files (x86)/Windows Kits/10/Include/10.0.26100.0/um/gl"
# INCLUDEPATH += "C:/Program Files (x86)/Windows Kits/10/Include/10.0.26100.0/um"
# INCLUDEPATH += "C:/Program Files (x86)/Windows Kits/10/Include/10.0.26100.0/shared"

# # OpenGL 库
# LIBS += -lopengl32 -lglu32

# # 链接 CUDA 头文件和库目录
# INCLUDEPATH += $$"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v13.3/include"
# LIBS += -L$$"C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v13.3/lib/x64"
# LIBS += -lcudart -lcuda

TEMPLATE = app
CONFIG -= app_bundle
TARGET = 3DScan
# HEADERS += algorithm.h \
#     vtkvboactor.h

INCLUDEPATH +=C:\CUDA\v13.3\include  \
 C:\CUDA\v13.3\common\inc
LIBS +=-LC:\CUDA\v13.3/lib/x64 \
-lcudart \
-lcublas \
-lcufft
OTHER_FILES +=./algorithm.cu
# Cuda sources
CUDA_SOURCES+=./algorithm.cu
CUDA_SDK = C:\CUDA\v13.3
CUDA_DIR = C:\CUDA\v13.3
QMAKE_LIBDIR += $$CUDA_DIR/lib/x64
SYSTEM_TYPE = 64
#不同的显卡注意填适当的选项""
CUDA_ARCH = sm_86
NVCCFLAGS     = --use_fast_math
CUDA_INC = $$join("C:\CUDA\v13.3/include",'" -I"','-I"','"')
# # 构建 nvcc 的包含路径（使用 -I 参数）
# CUDA_INC = -I"C:/CUDA/v13.3/include" \
#            -I"C:/CUDA/v13.3/common/inc" \
#            -I"C:/OpenGL/Include/10.0.26100.0/um/gl" \
#            -I"C:/OpenGL/Include/10.0.26100.0/um" \
#            -I"C:/OpenGL/Include/10.0.26100.0/shared"

# MSVCRT link option (static or dynamic, it must be the same with your Qt SDK link option)
MSVCRT_LINK_FLAG_DEBUG = "/MDd"
MSVCRT_LINK_FLAG_RELEASE = "/MD"

# 配置编译器
CONFIG(debug, debug|release) {
CUDA_OBJECTS_DIR = ./debug
# Debug mode
cuda_d.input = CUDA_SOURCES
cuda_d.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}algorithm.obj
cuda_d.commands = \"$$CUDA_DIR/bin/nvcc.exe\" -D_DEBUG $$NVCC_OPTIONS $$CUDA_INC $$CUDA_LIBS --machine $$SYSTEM_TYPE \
                 -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME} -Xcompiler $$MSVCRT_LINK_FLAG_DEBUG
cuda_d.dependency_type = TYPE_C
QMAKE_EXTRA_COMPILERS += cuda_d
}
else {
CUDA_OBJECTS_DIR = ./release
# Release mode
 cuda.input = CUDA_SOURCES
 cuda.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}algorithm.obj
 cuda.commands = \"$$CUDA_DIR/bin/nvcc.exe\" $$NVCC_OPTIONS $$CUDA_INC $$CUDA_LIBS --machine $$SYSTEM_TYPE \
    -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME} -Xcompiler $$MSVCRT_LINK_FLAG_RELEASE
cuda.dependency_type = TYPE_C
 QMAKE_EXTRA_COMPILERS += cuda
}

# 链接 VTK 头文件和库目录
INCLUDEPATH += $$"C:/Program Files (x86)/VTK/include/vtk-9.6"
LIBS += -L$$"C:/Program Files (x86)/VTK/lib"

LIBS += -lvtkGUISupportQt-9.6 \
        -lvtkRenderingQt-9.6 \
        -lvtkRenderingOpenGL2-9.6 \
        -lvtkRenderingCore-9.6 \
        -lvtkRenderingAnnotation-9.6 \
        -lvtkRenderingFreeType-9.6 \
        -lvtkRenderingVolume-9.6 \
        -lvtkRenderingLabel-9.6 \
        -lvtkRenderingVolumeOpenGL2-9.6 \
        -lvtkInteractionStyle-9.6 \
        -lvtkInteractionWidgets-9.6 \
        -lvtkCommonCore-9.6 \
        -lvtkCommonMath-9.6 \
        -lvtkCommonDataModel-9.6 \
        -lvtkCommonExecutionModel-9.6 \
        -lvtkCommonTransforms-9.6 \
        -lvtkFiltersCore-9.6 \
        -lvtkFiltersSources-9.6 \
        -lvtkFiltersGeneral-9.6 \
        -lvtkFiltersModeling-9.6 \
        -lvtkFiltersGeometry-9.6 \
        -lvtkIOImage-9.6 \
        -lvtkIOPLY-9.6 \
        -lvtkImagingGeneral-9.6 \
        -lvtkImagingCore-9.6 \
        -lvtkImagingHybrid-9.6 \
        -lvtkImagingSources-9.6 \
        -lvtkViewsQt-9.6 \
        -lvtksys-9.6

# 链接 OpenGL 头文件和库目录
LIBS += -lopengl32

SOURCES += \
    main.cpp \
    mainwindow3.cpp \
    datapanel.cpp \
    scan.cpp \
    scandata.cpp \
    vtkvboactor.cpp

HEADERS += \
    mainwindow3.h \
    datapanel.h \
    scan.h \
    scandata.h \
    algorithm.h \
    vtkvboactor.h

FORMS += \
    mainwindow3.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


