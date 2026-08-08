// algorithm.cu
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#include <GL/gl.h>
#include <GL/glu.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "algorithm.h"

__global__ void transformPointsVBO_Kernel(
    const float* pose,
    const float* amp,
    const float* tof,
    float si,
    int beam,
    int centerBeam,
    float spacing,
    int isAmpMode,
    float* vboPoints,
    unsigned char* vboColors
    )
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= beam) return;
    if (amp[idx] == 0.0f || tof[idx] == 0.0f) return;

    // ===== 坐标变换（与VTK保持一致）=====
    float offset = (idx - centerBeam) * spacing;
    float localX = 0.0f;
    float localY = offset;
    float localZ = -si * 0.5f;

    // 注意：pose[3]=Roll, pose[4]=Pitch, pose[5]=Yaw
    float roll = pose[3];   // 对应VTK的RotateX
    float pitch = pose[4];  // 对应VTK的RotateY
    float yaw = pose[5];    // 对应VTK的RotateZ

    // ===== VTK变换顺序：先Z(-yaw) → 再Y(pitch) → 再X(roll) =====

    // 第一步：绕Z轴旋转（负偏航角），对应 VTK 的 RotateZ(-pose[5])
    float cosY = cosf(-yaw), sinY = sinf(-yaw);
    float x1 = localX * cosY - localY * sinY;
    float y1 = localX * sinY + localY * cosY;
    float z1 = localZ;

    // 第二步：绕Y轴旋转（俯仰角），对应 VTK 的 RotateY(pose[4])
    float cosP = cosf(pitch), sinP = sinf(pitch);
    float x2 = x1 * cosP + z1 * sinP;
    float y2 = y1;
    float z2 = -x1 * sinP + z1 * cosP;

    // 第三步：绕X轴旋转（滚转角），对应 VTK 的 RotateX(pose[3])
    float cosR = cosf(roll), sinR = sinf(roll);
    float x3 = x2;
    float y3 = y2 * cosR - z2 * sinR;
    float z3 = y2 * sinR + z2 * cosR;

    // 平移，对应 VTK 的 TransformPoint 后的平移
    float worldX = x3 + pose[0];
    float worldY = y3 + pose[1];
    float worldZ = z3 + pose[2];

    // 直接写入 VBO (xyz 交错)
    int outIdx = idx * 3;
    vboPoints[outIdx] = worldX;
    vboPoints[outIdx + 1] = worldY;
    vboPoints[outIdx + 2] = worldZ;

    // ===== 颜色映射（保持不变）=====
    float value = isAmpMode ? amp[idx] : tof[idx];
    float normalized = fminf(1.0f, fmaxf(0.0f, value / 255.0f));

    unsigned char red, green, blue;
    if (normalized < 0.5f) {
        float t = normalized / 0.5f;
        red = 0;
        green = (unsigned char)(t * 255);
        blue = 255;
    } else {
        float t = (normalized - 0.5f) / 0.5f;
        red = (unsigned char)(t * 255);
        green = (unsigned char)((1.0f - t) * 255);
        blue = 0;
    }

    // 直接写入 VBO 颜色 (rgb 交错)
    int colorIdx = idx * 3;
    vboColors[colorIdx] = red;
    vboColors[colorIdx + 1] = green;
    vboColors[colorIdx + 2] = blue;
}

// ============ CUDA 处理器 (VBO 直写) ============
class CUDAPointCloudProcessor
{
public:
    CUDAPointCloudProcessor(int maxBeamSize);
    ~CUDAPointCloudProcessor();

    // VBO 管理
    int registerVBO(GLuint vboPoints, GLuint vboColors, int maxPoints);
    int unregisterVBO();

    // 直接处理写入 VBO
    int processDirect(const double pose[], const double amp[], const double tof[],
                      double si, int beam, int isAmpMode);

    bool isInitialized() const { return initialized; }
    bool isVBORegistered() const { return vboRegistered; }

private:
    void cleanup();

    int maxBeam = 0;
    int vboMaxPoints = 0;
    bool initialized = false;
    bool vboRegistered = false;

    // 主机固定内存 (CPU快速传输)
    float* h_pose = nullptr;
    float* h_amp = nullptr;
    float* h_tof = nullptr;

    // 设备内存 (用于异步传输)
    float* d_pose = nullptr;
    float* d_amp = nullptr;
    float* d_tof = nullptr;

    // CUDA-OpenGL 互操作资源
    cudaGraphicsResource* cudaVboPoints = nullptr;
    cudaGraphicsResource* cudaVboColors = nullptr;

    // 映射后的设备指针
    float* d_vboPoints = nullptr;
    unsigned char* d_vboColors = nullptr;
};

CUDAPointCloudProcessor::CUDAPointCloudProcessor(int maxBeamSize)
{
    if (maxBeamSize <= 0) return;

    int deviceCount = 0;
    if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) return;

    // 使用固定内存加速传输
    cudaMallocHost(&h_pose, 6 * sizeof(float));
    cudaMallocHost(&h_amp, maxBeamSize * sizeof(float));
    cudaMallocHost(&h_tof, maxBeamSize * sizeof(float));

    if (!h_pose || !h_amp || !h_tof) { cleanup(); return; }

    // 分配设备内存
    cudaError_t err;
    err = cudaMalloc(&d_pose, 6 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_amp, maxBeamSize * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_tof, maxBeamSize * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }

    maxBeam = maxBeamSize;
    initialized = true;

    printf("CUDA Processor initialized, maxBeam: %d\n", maxBeam);
}

CUDAPointCloudProcessor::~CUDAPointCloudProcessor()
{
    unregisterVBO();
    cleanup();
}

void CUDAPointCloudProcessor::cleanup()
{
    if (h_pose) { cudaFreeHost(h_pose); h_pose = nullptr; }
    if (h_amp) { cudaFreeHost(h_amp); h_amp = nullptr; }
    if (h_tof) { cudaFreeHost(h_tof); h_tof = nullptr; }

    if (d_pose) { cudaFree(d_pose); d_pose = nullptr; }
    if (d_amp) { cudaFree(d_amp); d_amp = nullptr; }
    if (d_tof) { cudaFree(d_tof); d_tof = nullptr; }

    maxBeam = 0;
    initialized = false;
}

int CUDAPointCloudProcessor::registerVBO(GLuint vboPoints, GLuint vboColors, int maxPoints)
{
    if (!initialized) {
        printf("ERROR: Processor not initialized\n");
        return -1;
    }

    // 注销旧的 VBO
    unregisterVBO();

    vboMaxPoints = maxPoints;

    // 注册 VBO 到 CUDA
    cudaError_t err;
    err = cudaGraphicsGLRegisterBuffer(&cudaVboPoints, vboPoints,
                                       cudaGraphicsMapFlagsWriteDiscard);
    if (err != cudaSuccess) {
        printf("Failed to register VBO points: %s\n", cudaGetErrorString(err));
        return -1;
    }

    err = cudaGraphicsGLRegisterBuffer(&cudaVboColors, vboColors,
                                       cudaGraphicsMapFlagsWriteDiscard);
    if (err != cudaSuccess) {
        printf("Failed to register VBO colors: %s\n", cudaGetErrorString(err));
        cudaGraphicsUnregisterResource(cudaVboPoints);
        cudaVboPoints = nullptr;
        return -1;
    }

    vboRegistered = true;
    printf("VBO registered successfully, maxPoints: %d\n", maxPoints);
    return 0;
}

int CUDAPointCloudProcessor::unregisterVBO()
{
    if (cudaVboPoints) {
        cudaGraphicsUnregisterResource(cudaVboPoints);
        cudaVboPoints = nullptr;
    }
    if (cudaVboColors) {
        cudaGraphicsUnregisterResource(cudaVboColors);
        cudaVboColors = nullptr;
    }
    d_vboPoints = nullptr;
    d_vboColors = nullptr;
    vboRegistered = false;
    return 0;
}

int CUDAPointCloudProcessor::processDirect(const double pose[], const double amp[],
                                           const double tof[], double si, int beam,
                                           int isAmpMode)
{
    if (!initialized) {
        printf("ERROR: Processor not initialized\n");
        return -1;
    }
    if (beam <= 0 || beam > maxBeam) {
        printf("ERROR: beam %d invalid (max: %d)\n", beam, maxBeam);
        return -1;
    }
    if (!pose || !amp || !tof) {
        printf("ERROR: Null input pointer\n");
        return -1;
    }
    if (!vboRegistered) {
        printf("ERROR: VBO not registered\n");
        return -1;
    }
    if (beam > vboMaxPoints) {
        printf("ERROR: beam %d exceeds VBO max %d\n", beam, vboMaxPoints);
        return -1;
    }

    // ===== 1. 映射 VBO 到 CUDA 地址空间 =====
    cudaError_t err;
    err = cudaGraphicsMapResources(1, &cudaVboPoints, 0);
    if (err != cudaSuccess) {
        printf("Map VBO points failed: %s\n", cudaGetErrorString(err));
        return -1;
    }
    err = cudaGraphicsMapResources(1, &cudaVboColors, 0);
    if (err != cudaSuccess) {
        printf("Map VBO colors failed: %s\n", cudaGetErrorString(err));
        cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
        return -1;
    }

    // 获取映射后的指针
    size_t pointSize = 0, colorSize = 0;
    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboPoints, &pointSize, cudaVboPoints);
    if (err != cudaSuccess) {
        printf("Get VBO points pointer failed: %s\n", cudaGetErrorString(err));
        cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
        cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
        return -1;
    }

    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboColors, &colorSize, cudaVboColors);
    if (err != cudaSuccess) {
        printf("Get VBO colors pointer failed: %s\n", cudaGetErrorString(err));
        cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
        cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
        return -1;
    }

    // ===== 2. 复制数据到设备 =====
    for (int i = 0; i < 6; i++) h_pose[i] = (float)pose[i];
    for (int i = 0; i < beam; i++) {
        h_amp[i] = (float)amp[i];
        h_tof[i] = (float)tof[i];
    }

    err = cudaMemcpy(d_pose, h_pose, 6 * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        printf("Copy pose failed: %s\n", cudaGetErrorString(err));
        goto cleanup;
    }

    err = cudaMemcpy(d_amp, h_amp, beam * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        printf("Copy amp failed: %s\n", cudaGetErrorString(err));
        goto cleanup;
    }

    err = cudaMemcpy(d_tof, h_tof, beam * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        printf("Copy tof failed: %s\n", cudaGetErrorString(err));
        goto cleanup;
    }

    // ===== 3. 启动内核 - 直接写入 VBO =====
    int threadsPerBlock = 256;
    int blocksPerGrid = (beam + threadsPerBlock - 1) / threadsPerBlock;
    int centerBeam = (beam - 1) / 2;
    float spacing = 0.3f;

    transformPointsVBO_Kernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_pose, d_amp, d_tof, (float)si, beam, centerBeam, spacing,
        isAmpMode,
        d_vboPoints,   // 直接写入 VBO
        d_vboColors    // 直接写入 VBO
        );

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        printf("Kernel failed: %s\n", cudaGetErrorString(err));
    }

cleanup:
    // ===== 4. 取消映射 VBO =====
    cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
    cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
    d_vboPoints = nullptr;
    d_vboColors = nullptr;

    return (err == cudaSuccess) ? 0 : -1;
}

// ============ C 接口 ============
extern "C" {

CUDAProcessorHandle createCUDAProcessor(int maxBeamSize)
{
    if (maxBeamSize <= 0) maxBeamSize = 64;
    CUDAPointCloudProcessor* processor = new CUDAPointCloudProcessor(maxBeamSize);
    if (!processor->isInitialized()) {
        delete processor;
        return nullptr;
    }
    return (CUDAProcessorHandle)processor;
}

void destroyCUDAProcessor(CUDAProcessorHandle processor)
{
    if (processor) {
        delete (CUDAPointCloudProcessor*)processor;
    }
}

int registerVBO(CUDAProcessorHandle processor,
                unsigned int vboPoints,
                unsigned int vboColors,
                int maxPoints)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->registerVBO(
        vboPoints, vboColors, maxPoints);
}

int unregisterVBO(CUDAProcessorHandle processor)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->unregisterVBO();
}

int processDirectVBO(CUDAProcessorHandle processor,
                     const double* pose,
                     const double* amp,
                     const double* tof,
                     double si,
                     int beam,
                     int isAmpMode)
{      
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->processDirect(
        pose, amp, tof, si, beam, isAmpMode);
}

int isCUDAAvailable(void)
{
    int deviceCount = 0;
    return (cudaGetDeviceCount(&deviceCount) == cudaSuccess && deviceCount > 0) ? 1 : 0;
}

} // extern "C"
