# -----------------------------------------------------------------------------
# 3DScan - Qt / VTK / CUDA project configuration
# -----------------------------------------------------------------------------

TEMPLATE = app
TARGET = 3DScan
CONFIG -= app_bundle
CONFIG += c++17 force_debug_info

QT += core gui widgets network opengl openglwidgets

msvc: QMAKE_CXXFLAGS += /MP

# Keep generated Makefiles independent of the depth of the build directory.
QMAKE_PROJECT_DEPTH = 0

# -----------------------------------------------------------------------------
# CUDA
# -----------------------------------------------------------------------------

CUDA_DIR = $$quote(C:/Program Files/NVIDIA GPU Computing Toolkit/CUDA/v13.3)
CUDA_SDK = $$CUDA_DIR
SYSTEM_TYPE = 64
CUDA_ARCH = sm_86
NVCCFLAGS = --use_fast_math

INCLUDEPATH += $$CUDA_DIR/include
QMAKE_LIBDIR += $$CUDA_DIR/lib/x64

LIBS += -L$$quote($$CUDA_DIR/lib/x64) \
        -lcudart \
        -lcublas \
        -lcufft

CUDA_SOURCES += algorithm.cu
OTHER_FILES += algorithm.cu

# The escaped quotes keep the CUDA include path intact when it contains spaces.
CUDA_INC = -I\"$$CUDA_DIR/include\"

MSVCRT_LINK_FLAG_DEBUG = "/MDd"
MSVCRT_LINK_FLAG_RELEASE = "/MD"

CONFIG(debug, debug|release) {
    CUDA_OBJECTS_DIR = ./debug

    cuda_d.input = CUDA_SOURCES
    cuda_d.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}algorithm.obj
    cuda_d.commands = \"$$CUDA_DIR/bin/nvcc.exe\" -D_DEBUG $$NVCC_OPTIONS $$CUDA_INC $$CUDA_LIBS --machine $$SYSTEM_TYPE \
                     -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME} \
                     -Xcompiler $$MSVCRT_LINK_FLAG_DEBUG
    cuda_d.dependency_type = TYPE_C
    QMAKE_EXTRA_COMPILERS += cuda_d
} else {
    CUDA_OBJECTS_DIR = ./release

    cuda.input = CUDA_SOURCES
    cuda.output = $$CUDA_OBJECTS_DIR/${QMAKE_FILE_BASE}algorithm.obj
    cuda.commands = \"$$CUDA_DIR/bin/nvcc.exe\" $$NVCC_OPTIONS $$CUDA_INC $$CUDA_LIBS --machine $$SYSTEM_TYPE \
                    -arch=$$CUDA_ARCH -c -o ${QMAKE_FILE_OUT} ${QMAKE_FILE_NAME} \
                    -Xcompiler $$MSVCRT_LINK_FLAG_RELEASE
    cuda.dependency_type = TYPE_C
    QMAKE_EXTRA_COMPILERS += cuda
}

# -----------------------------------------------------------------------------
# VTK
# -----------------------------------------------------------------------------

VTK_DIR = C:/Program Files (x86)/VTK
VTK_INCLUDE_DIR = $$quote($$VTK_DIR/include/vtk-9.6)
VTK_LIBRARY_DIR = $$quote($$VTK_DIR/lib)

INCLUDEPATH += $$VTK_INCLUDE_DIR
LIBS += -L$$VTK_LIBRARY_DIR

LIBS += \
    -lvtkGUISupportQt-9.6 \
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

# -----------------------------------------------------------------------------
# OpenGL
# -----------------------------------------------------------------------------

LIBS += -lopengl32

# -----------------------------------------------------------------------------
# Project files
# -----------------------------------------------------------------------------

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

FORMS += mainwindow3.ui

# -----------------------------------------------------------------------------
# Deployment
# -----------------------------------------------------------------------------

qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
