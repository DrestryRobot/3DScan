// algorithm.cu - CUDA point transform and VBO write
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

// ---------------------------------------------------------------------------
// Kernel: transform beam points into world space and append them to the VBO.
// Only valid points (amp != 0 && tof != 0) are written. A device counter is
// used to compact valid writes into a contiguous cloud, so the accumulated
// point cloud contains no gaps and no stale points.
// ---------------------------------------------------------------------------
__global__ void transformPointsVBO_Kernel(
    const float* __restrict__ pose,
    const float* __restrict__ amp,
    const float* __restrict__ tof,
    const float* __restrict__ localZ,
    int beam,
    int centerBeam,
    float spacing,
    int isAmpMode,
    int startValid,
    int maxPoints,
    int* __restrict__ d_validCounter,
    const unsigned int* __restrict__ d_colorLUT,
    int lutSize,
    float* vboPoints,
    unsigned char* vboColors)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= beam) return;
    if (amp[idx] == 0.0f || tof[idx] == 0.0f) return;

    // 49 elements lie along the probe's local Y axis (KUKA tool frame).
    float offset = (idx - centerBeam) * spacing;
    float localX = 0.0f;
    float localY = offset;
    float localZVal = localZ ? localZ[idx] : 0.0f;

    // KUKA convention: A=yaw(Z), B=pitch(Y), C=roll(X), all in degrees.
    // R = Rz(A) * Ry(B) * Rx(C)  -> apply X(C), then Y(B), then Z(A).
    const float deg2rad = 0.01745329252f;
    float angA = pose[3] * deg2rad;   // Z rotation
    float angB = pose[4] * deg2rad;   // Y rotation
    float angC = pose[5] * deg2rad;   // X rotation

    // X rotation by C
    float cosR = cosf(angC), sinR = sinf(angC);
    float x1 = localX;
    float y1 = localY * cosR - localZVal * sinR;
    float z1 = localY * sinR + localZVal * cosR;

    // Y rotation by B
    float cosP = cosf(angB), sinP = sinf(angB);
    float x2 = x1 * cosP + z1 * sinP;
    float y2 = y1;
    float z2 = -x1 * sinP + z1 * cosP;

    // Z rotation by A
    float cosY = cosf(angA), sinY = sinf(angA);
    float x3 = x2 * cosY - y2 * sinY;
    float y3 = x2 * sinY + y2 * cosY;
    float z3 = z2;

    float worldX = x3 + pose[0];
    float worldY = y3 + pose[1];
    float worldZ = z3 + pose[2];

    // ---- compact append ----
    int pos = startValid + atomicAdd(d_validCounter, 1);
    if (pos >= maxPoints) return;

    int outIdx = pos * 3;
    vboPoints[outIdx]     = worldX;
    vboPoints[outIdx + 1] = worldY;
    vboPoints[outIdx + 2] = worldZ;

    // ---- color: same 256-level LUT as the CPU color bar ----
    float value     = isAmpMode ? amp[idx] : tof[idx];
    float normalized = fminf(1.0f, fmaxf(0.0f, value));

    unsigned char red, green, blue;
    if (d_colorLUT && lutSize > 0) {
        int ci = (int)(normalized * (lutSize - 1));
        if (ci < 0) ci = 0;
        if (ci >= lutSize) ci = lutSize - 1;
        unsigned int color = d_colorLUT[ci];
        red   = (unsigned char)((color >> 16) & 0xFF);
        green = (unsigned char)((color >> 8) & 0xFF);
        blue  = (unsigned char)(color & 0xFF);
    } else {
        // fallback gradient (data is normalized 0..1)
        if (normalized < 0.5f) {
            float t = normalized / 0.5f;
            red   = 0;
            green = (unsigned char)(t * 255);
            blue  = 255;
        } else {
            float t = (normalized - 0.5f) / 0.5f;
            red   = (unsigned char)(t * 255);
            green = (unsigned char)((1.0f - t) * 255);
            blue  = 0;
        }
    }

    vboColors[outIdx]     = red;
    vboColors[outIdx + 1] = green;
    vboColors[outIdx + 2] = blue;
}

// Re-color an existing accumulated point cloud (AMP or TOF values).
__global__ void recolorPointsKernel(
    const float* __restrict__ values,
    int count,
    const unsigned int* __restrict__ d_colorLUT,
    int lutSize,
    unsigned char* vboColors)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count) return;

    float normalized = fminf(1.0f, fmaxf(0.0f, values[i]));
    unsigned char red, green, blue;
    if (d_colorLUT && lutSize > 0) {
        int ci = (int)(normalized * (lutSize - 1));
        if (ci < 0) ci = 0;
        if (ci >= lutSize) ci = lutSize - 1;
        unsigned int color = d_colorLUT[ci];
        red   = (unsigned char)((color >> 16) & 0xFF);
        green = (unsigned char)((color >> 8) & 0xFF);
        blue  = (unsigned char)(color & 0xFF);
    } else {
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
    }

    int outIdx = i * 3;
    vboColors[outIdx]     = red;
    vboColors[outIdx + 1] = green;
    vboColors[outIdx + 2] = blue;
}

// ---------------------------------------------------------------------------
// Batch version of transformPointsVBO_Kernel: process frameCount frames in a
// single kernel launch (one map/kernel/unmap/sync instead of per-frame).
// Inputs are flattened: poses[f*6], amps/tofs/localZs[f*beam + e].
// ---------------------------------------------------------------------------
__global__ void transformPointsVBatch_Kernel(
    const float* __restrict__ poses,
    const float* __restrict__ amps,
    const float* __restrict__ tofs,
    const float* __restrict__ localZs,
    int beam,
    int frameCount,
    int centerBeam,
    float spacing,
    int isAmpMode,
    int startValid,
    int maxPoints,
    int* __restrict__ d_validCounter,
    const unsigned int* __restrict__ d_colorLUT,
    int lutSize,
    float* vboPoints,
    unsigned char* vboColors)
{
    int total = beam * frameCount;
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= total) return;

    int f = idx / beam;
    int e = idx - f * beam;
    const float* pose = poses + f * 6;
    float a = amps[idx];
    float t = tofs[idx];
    if (a == 0.0f || t == 0.0f) return;

    float offset = (e - centerBeam) * spacing;
    float localX = 0.0f;
    float localY = offset;
    float localZVal = localZs ? localZs[idx] : 0.0f;

    const float deg2rad = 0.01745329252f;
    float angA = pose[3] * deg2rad;   // Z rotation
    float angB = pose[4] * deg2rad;   // Y rotation
    float angC = pose[5] * deg2rad;   // X rotation

    float cosR = cosf(angC), sinR = sinf(angC);
    float x1 = localX;
    float y1 = localY * cosR - localZVal * sinR;
    float z1 = localY * sinR + localZVal * cosR;

    float cosP = cosf(angB), sinP = sinf(angB);
    float x2 = x1 * cosP + z1 * sinP;
    float y2 = y1;
    float z2 = -x1 * sinP + z1 * cosP;

    float cosY = cosf(angA), sinY = sinf(angA);
    float x3 = x2 * cosY - y2 * sinY;
    float y3 = x2 * sinY + y2 * cosY;
    float z3 = z2;

    float worldX = x3 + pose[0];
    float worldY = y3 + pose[1];
    float worldZ = z3 + pose[2];

    int pos = startValid + atomicAdd(d_validCounter, 1);
    if (pos >= maxPoints) return;

    int outIdx = pos * 3;
    vboPoints[outIdx]     = worldX;
    vboPoints[outIdx + 1] = worldY;
    vboPoints[outIdx + 2] = worldZ;

    float value     = isAmpMode ? a : t;
    float normalized = fminf(1.0f, fmaxf(0.0f, value));

    unsigned char red, green, blue;
    if (d_colorLUT && lutSize > 0) {
        int ci = (int)(normalized * (lutSize - 1));
        if (ci < 0) ci = 0;
        if (ci >= lutSize) ci = lutSize - 1;
        unsigned int color = d_colorLUT[ci];
        red   = (unsigned char)((color >> 16) & 0xFF);
        green = (unsigned char)((color >> 8) & 0xFF);
        blue  = (unsigned char)(color & 0xFF);
    } else {
        if (normalized < 0.5f) {
            float tt = normalized / 0.5f;
            red = 0; green = (unsigned char)(tt * 255); blue = 255;
        } else {
            float tt = (normalized - 0.5f) / 0.5f;
            red = (unsigned char)(tt * 255); green = (unsigned char)((1.0f - tt) * 255); blue = 0;
        }
    }
    vboColors[outIdx]     = red;
    vboColors[outIdx + 1] = green;
    vboColors[outIdx + 2] = blue;
}

// ---------------------------------------------------------------------------
// Write precomputed world-space points directly into the VBO (方案2 uniform grid).
__global__ void writeCloudPointsKernel(
    const float* __restrict__ worldXYZ,
    const float* __restrict__ amp,
    const float* __restrict__ tof,
    int count,
    int isAmpMode,
    int startValid,
    int maxPoints,
    int* __restrict__ d_validCounter,
    const unsigned int* __restrict__ d_colorLUT,
    int lutSize,
    float* vboPoints,
    unsigned char* vboColors)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= count) return;
    if (amp[i] == 0.0f || tof[i] == 0.0f) return;

    int pos = startValid + atomicAdd(d_validCounter, 1);
    if (pos >= maxPoints) return;

    int outIdx = pos * 3;
    vboPoints[outIdx]     = worldXYZ[i*3+0];
    vboPoints[outIdx + 1] = worldXYZ[i*3+1];
    vboPoints[outIdx + 2] = worldXYZ[i*3+2];

    float value = isAmpMode ? amp[i] : tof[i];
    float normalized = fminf(1.0f, fmaxf(0.0f, value));
    unsigned char red, green, blue;
    if (d_colorLUT && lutSize > 0) {
        int ci = (int)(normalized * (lutSize - 1));
        if (ci < 0) ci = 0;
        if (ci >= lutSize) ci = lutSize - 1;
        unsigned int color = d_colorLUT[ci];
        red   = (unsigned char)((color >> 16) & 0xFF);
        green = (unsigned char)((color >> 8) & 0xFF);
        blue  = (unsigned char)(color & 0xFF);
    } else {
        if (normalized < 0.5f) {
            float t = normalized / 0.5f;
            red = 0; green = (unsigned char)(t * 255); blue = 255;
        } else {
            float t = (normalized - 0.5f) / 0.5f;
            red = (unsigned char)(t * 255); green = (unsigned char)((1.0f - t) * 255); blue = 0;
        }
    }
    vboColors[outIdx]     = red;
    vboColors[outIdx + 1] = green;
    vboColors[outIdx + 2] = blue;
}
// CUDAPointCloudProcessor: owns pinned/device buffers and the CUDA-GL interop
// ---------------------------------------------------------------------------
class CUDAPointCloudProcessor
{
public:
    CUDAPointCloudProcessor(int maxBeamSize);
    ~CUDAPointCloudProcessor();

    int registerVBO(GLuint vboPoints, GLuint vboColors, int maxPoints);
    int unregisterVBO();
    int resetVBO();
    int setColorLUT(const unsigned int* lut, int size);
    int recolorVBO(const float* values, int count);

    int processDirect(const double pose[], const double amp[], const double tof[],
                      const double localZ[], int beam, int isAmpMode, int startValid);
    int processCloud(const float worldXYZ[], const double amp[], const double tof[],
                     int count, int isAmpMode, int startValid);
    // 批量版：一次 map/kernel/unmap/sync 处理 frameCount 帧
    int processDirectBatch(const double poses[], const double amps[],
                           const double tofs[], const double localZs[],
                           int beam, int frameCount, int isAmpMode, int startValid);
    int processCloudBatch(const float worldXYZ[], const double amps[],
                          const double tofs[], int totalCount,
                          int isAmpMode, int startValid);

    bool isInitialized() const { return initialized; }
    bool isVBORegistered() const { return vboRegistered; }

private:
    void cleanup();

    int maxBeam = 0;
    int vboMaxPoints = 0;
    bool initialized = false;
    bool vboRegistered = false;

    float* h_pose = nullptr;
    float* h_amp  = nullptr;
    float* h_tof  = nullptr;
    float* h_localZ = nullptr;

    float* d_pose = nullptr;
    float* d_amp  = nullptr;
    float* d_tof  = nullptr;
    float* d_localZ = nullptr;
    float* h_worldXYZ = nullptr;
    float* d_worldXYZ = nullptr;

    // 批量缓冲区（CUDA_BATCH_MAX 帧，攒批一次同步）
    float* h_batchPose = nullptr;
    float* h_batchAmp  = nullptr;
    float* h_batchTof  = nullptr;
    float* h_batchLocalZ = nullptr;
    float* h_batchWorld  = nullptr;
    float* d_batchPose = nullptr;
    float* d_batchAmp  = nullptr;
    float* d_batchTof  = nullptr;
    float* d_batchLocalZ = nullptr;
    float* d_batchWorld  = nullptr;

    int* d_validCounter = nullptr;
    unsigned int* d_colorLUT = nullptr;
    int lutSize = 0;
    float* d_recolor = nullptr;

    cudaGraphicsResource* cudaVboPoints = nullptr;
    cudaGraphicsResource* cudaVboColors = nullptr;

    float* d_vboPoints = nullptr;
    unsigned char* d_vboColors = nullptr;
};

CUDAPointCloudProcessor::CUDAPointCloudProcessor(int maxBeamSize)
{
    if (maxBeamSize <= 0) return;

    int deviceCount = 0;
    if (cudaGetDeviceCount(&deviceCount) != cudaSuccess || deviceCount == 0) return;

    cudaMallocHost(&h_pose, 6 * sizeof(float));
    cudaMallocHost(&h_amp, maxBeamSize * sizeof(float));
    cudaMallocHost(&h_tof, maxBeamSize * sizeof(float));
    cudaMallocHost(&h_localZ, maxBeamSize * sizeof(float));
    if (!h_pose || !h_amp || !h_tof || !h_localZ) { cleanup(); return; }

    cudaError_t err;
    err = cudaMalloc(&d_pose, 6 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_amp, maxBeamSize * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_tof, maxBeamSize * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_localZ, maxBeamSize * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMallocHost(&h_worldXYZ, 3 * maxBeamSize * sizeof(float));
    if (!h_worldXYZ) { cleanup(); return; }
    err = cudaMalloc(&d_worldXYZ, 3 * maxBeamSize * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }

    const int kBatch = CUDA_BATCH_MAX;
    err = cudaMallocHost(&h_batchPose, kBatch * 6 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMallocHost(&h_batchAmp, kBatch * 64 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMallocHost(&h_batchTof, kBatch * 64 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMallocHost(&h_batchLocalZ, kBatch * 64 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMallocHost(&h_batchWorld, kBatch * 64 * 3 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_batchPose, kBatch * 6 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_batchAmp, kBatch * 64 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_batchTof, kBatch * 64 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_batchLocalZ, kBatch * 64 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_batchWorld, kBatch * 64 * 3 * sizeof(float));
    if (err != cudaSuccess) { cleanup(); return; }

    err = cudaMalloc(&d_validCounter, sizeof(int));
    if (err != cudaSuccess) { cleanup(); return; }
    err = cudaMalloc(&d_colorLUT, 256 * sizeof(unsigned int));
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
    if (h_amp)  { cudaFreeHost(h_amp);  h_amp  = nullptr; }
    if (h_tof)  { cudaFreeHost(h_tof);  h_tof  = nullptr; }
    if (h_localZ) { cudaFreeHost(h_localZ); h_localZ = nullptr; }
    if (h_worldXYZ) { cudaFreeHost(h_worldXYZ); h_worldXYZ = nullptr; }

    if (h_batchPose)   { cudaFreeHost(h_batchPose);   h_batchPose   = nullptr; }
    if (h_batchAmp)    { cudaFreeHost(h_batchAmp);    h_batchAmp    = nullptr; }
    if (h_batchTof)    { cudaFreeHost(h_batchTof);    h_batchTof    = nullptr; }
    if (h_batchLocalZ) { cudaFreeHost(h_batchLocalZ); h_batchLocalZ = nullptr; }
    if (h_batchWorld)  { cudaFreeHost(h_batchWorld);  h_batchWorld  = nullptr; }

    if (d_pose) { cudaFree(d_pose); d_pose = nullptr; }
    if (d_amp)  { cudaFree(d_amp);  d_amp  = nullptr; }
    if (d_tof)  { cudaFree(d_tof);  d_tof  = nullptr; }
    if (d_localZ) { cudaFree(d_localZ); d_localZ = nullptr; }
    if (d_worldXYZ) { cudaFree(d_worldXYZ); d_worldXYZ = nullptr; }
    if (d_worldXYZ) { cudaFree(d_worldXYZ); d_worldXYZ = nullptr; }
    if (d_batchPose)   { cudaFree(d_batchPose);   d_batchPose   = nullptr; }
    if (d_batchAmp)    { cudaFree(d_batchAmp);    d_batchAmp    = nullptr; }
    if (d_batchTof)    { cudaFree(d_batchTof);    d_batchTof    = nullptr; }
    if (d_batchLocalZ) { cudaFree(d_batchLocalZ); d_batchLocalZ = nullptr; }
    if (d_batchWorld)  { cudaFree(d_batchWorld);  d_batchWorld  = nullptr; }
    if (d_validCounter) { cudaFree(d_validCounter); d_validCounter = nullptr; }
    if (d_colorLUT)     { cudaFree(d_colorLUT);     d_colorLUT     = nullptr; }
    if (d_recolor)      { cudaFree(d_recolor);      d_recolor      = nullptr; }

    maxBeam = 0;
    initialized = false;
}

int CUDAPointCloudProcessor::registerVBO(GLuint vboPoints, GLuint vboColors, int maxPoints)
{
    if (!initialized) {
        printf("ERROR: Processor not initialized\n");
        return -1;
    }

    unregisterVBO();

    vboMaxPoints = maxPoints;

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
    if (d_recolor) cudaFree(d_recolor);
    cudaMalloc(&d_recolor, (size_t)maxPoints * sizeof(float));
    return resetVBO();
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
    if (d_recolor) { cudaFree(d_recolor); d_recolor = nullptr; }
    vboRegistered = false;
    return 0;
}

int CUDAPointCloudProcessor::resetVBO()
{
    if (!vboRegistered || vboMaxPoints <= 0) return -1;

    cudaError_t err;
    err = cudaGraphicsMapResources(1, &cudaVboPoints, 0);
    if (err != cudaSuccess) return -1;
    err = cudaGraphicsMapResources(1, &cudaVboColors, 0);
    if (err != cudaSuccess) {
        cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
        return -1;
    }

    size_t pointSize = 0, colorSize = 0;
    cudaGraphicsResourceGetMappedPointer((void**)&d_vboPoints, &pointSize, cudaVboPoints);
    cudaGraphicsResourceGetMappedPointer((void**)&d_vboColors, &colorSize, cudaVboColors);

    cudaMemset(d_vboPoints, 0, (size_t)vboMaxPoints * 3 * sizeof(float));
    cudaMemset(d_vboColors, 0, (size_t)vboMaxPoints * 3 * sizeof(unsigned char));
    cudaMemset(d_validCounter, 0, sizeof(int));

    cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
    cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
    cudaDeviceSynchronize();

    d_vboPoints = nullptr;
    d_vboColors = nullptr;
    return 0;
}

int CUDAPointCloudProcessor::setColorLUT(const unsigned int* lut, int size)
{
    if (!initialized || !lut || size <= 0) return -1;
    if (size > 256) size = 256;
    cudaError_t err = cudaMemcpy(d_colorLUT, lut, (size_t)size * sizeof(unsigned int),
                                 cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        printf("Copy color LUT failed: %s\n", cudaGetErrorString(err));
        return -1;
    }
    lutSize = size;
    return 0;
}

int CUDAPointCloudProcessor::recolorVBO(const float* values, int count)
{
    if (!initialized || !vboRegistered || !values || count <= 0) return -1;
    if (count > vboMaxPoints) count = vboMaxPoints;
    if (!d_recolor) return -1;

    cudaError_t err = cudaMemcpy(d_recolor, values, (size_t)count * sizeof(float),
                                 cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
        printf("Copy recolor values failed: %s\n", cudaGetErrorString(err));
        return -1;
    }

    err = cudaGraphicsMapResources(1, &cudaVboColors, 0);
    if (err != cudaSuccess) return -1;

    size_t colorSize = 0;
    cudaGraphicsResourceGetMappedPointer((void**)&d_vboColors, &colorSize, cudaVboColors);

    int threadsPerBlock = 256;
    int blocksPerGrid = (count + threadsPerBlock - 1) / threadsPerBlock;
    recolorPointsKernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_recolor, count, d_colorLUT, lutSize, d_vboColors);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) printf("Recolor kernel failed: %s\n", cudaGetErrorString(err));

    cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
    d_vboColors = nullptr;
    return (err == cudaSuccess) ? 0 : -1;
}

int CUDAPointCloudProcessor::processDirect(const double pose[], const double amp[],
                                           const double tof[], const double localZ[], int beam,
                                           int isAmpMode, int startValid)
{
    if (!initialized) {
        printf("ERROR: Processor not initialized\n");
        return -1;
    }
    if (beam <= 0 || beam > maxBeam) {
        printf("ERROR: beam %d invalid (max: %d)\n", beam, maxBeam);
        return -1;
    }
    if (!pose || !amp || !tof || !localZ) {
        printf("ERROR: Null input pointer\n");
        return -1;
    }
    if (!vboRegistered) {
        printf("ERROR: VBO not registered\n");
        return -1;
    }
    if (startValid >= vboMaxPoints) {
        return -2;  // cloud full
    }

    int threadsPerBlock = 256;
    int blocksPerGrid = (beam + threadsPerBlock - 1) / threadsPerBlock;
    int centerBeam = (beam - 1) / 2;
    float spacing = 0.3f;

    // ---- 1. map VBOs ----
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

    // ---- 2. copy inputs (pinned host -> device) ----
    for (int i = 0; i < 6; i++) h_pose[i] = (float)pose[i];
    for (int i = 0; i < beam; i++) {
        h_amp[i] = (float)amp[i];
        h_tof[i] = (float)tof[i];
        h_localZ[i] = (float)localZ[i];
    }

    err = cudaMemcpy(d_pose, h_pose, 6 * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { printf("Copy pose failed: %s\n", cudaGetErrorString(err)); goto cleanup; }
    err = cudaMemcpy(d_amp, h_amp, beam * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { printf("Copy amp failed: %s\n", cudaGetErrorString(err)); goto cleanup; }
    err = cudaMemcpy(d_tof, h_tof, beam * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { printf("Copy tof failed: %s\n", cudaGetErrorString(err)); goto cleanup; }
    err = cudaMemcpy(d_localZ, h_localZ, beam * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) { printf("Copy localZ failed: %s\n", cudaGetErrorString(err)); goto cleanup; }

    // reset the per-frame valid-point counter
    cudaMemset(d_validCounter, 0, sizeof(int));

    // ---- 3. launch kernel (append into accumulated cloud) ----

    transformPointsVBO_Kernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_pose, d_amp, d_tof, d_localZ, beam, centerBeam, spacing,
        isAmpMode, startValid, vboMaxPoints,
        d_validCounter, d_colorLUT, lutSize,
        d_vboPoints, d_vboColors);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        printf("Kernel failed: %s\n", cudaGetErrorString(err));
    }

cleanup:
    // ---- 4. unmap VBOs ----
    cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
    cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
    d_vboPoints = nullptr;
    d_vboColors = nullptr;

    return (err == cudaSuccess) ? 0 : -1;
}

// ---------------------------------------------------------------------------
int CUDAPointCloudProcessor::processCloud(const float worldXYZ[], const double amp[],
                                          const double tof[], int count,
                                          int isAmpMode, int startValid)
{
    if (!initialized) {
        printf("ERROR: Processor not initialized\n");
        return -1;
    }
    if (count <= 0 || count > maxBeam) {
        printf("ERROR: count %d invalid (max: %d)\n", count, maxBeam);
        return -1;
    }
    if (!worldXYZ || !amp || !tof) {
        printf("ERROR: Null input pointer\n");
        return -1;
    }
    if (!vboRegistered) {
        printf("ERROR: VBO not registered\n");
        return -1;
    }
    if (startValid >= vboMaxPoints) {
        return -2;  // cloud full
    }

    cudaError_t err;
    err = cudaGraphicsMapResources(1, &cudaVboPoints, 0);
    if (err != cudaSuccess) return -1;
    err = cudaGraphicsMapResources(1, &cudaVboColors, 0);
    if (err != cudaSuccess) {
        cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
        return -1;
    }

    size_t pointSize = 0, colorSize = 0;
    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboPoints, &pointSize, cudaVboPoints);
    if (err != cudaSuccess) goto cleanup;
    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboColors, &colorSize, cudaVboColors);
    if (err != cudaSuccess) goto cleanup;

    for (int i = 0; i < count; i++) {
        h_worldXYZ[i*3+0] = worldXYZ[i*3+0];
        h_worldXYZ[i*3+1] = worldXYZ[i*3+1];
        h_worldXYZ[i*3+2] = worldXYZ[i*3+2];
        h_amp[i] = (float)amp[i];
        h_tof[i] = (float)tof[i];
    }

    err = cudaMemcpy(d_worldXYZ, h_worldXYZ, (size_t)count * 3 * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_amp, h_amp, (size_t)count * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_tof, h_tof, (size_t)count * sizeof(float), cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;

    cudaMemset(d_validCounter, 0, sizeof(int));

    int threadsPerBlock = 256;
    int blocksPerGrid = (count + threadsPerBlock - 1) / threadsPerBlock;
    writeCloudPointsKernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_worldXYZ, d_amp, d_tof, count, isAmpMode, startValid, vboMaxPoints,
        d_validCounter, d_colorLUT, lutSize, d_vboPoints, d_vboColors);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        printf("writeCloudPointsKernel failed: %s\n", cudaGetErrorString(err));
    }

cleanup:
    cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
    cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
    d_vboPoints = nullptr;
    d_vboColors = nullptr;
    return (err == cudaSuccess) ? 0 : -1;
}

// ---------------------------------------------------------------------------
// 批量 pose 变换路径：一次处理 frameCount 帧（每帧 beam 个波束），
// 一次 map / 一次 kernel / 一次 unmap / 一次同步。
// ---------------------------------------------------------------------------
int CUDAPointCloudProcessor::processDirectBatch(
    const double poses[], const double amps[], const double tofs[],
    const double localZs[], int beam, int frameCount,
    int isAmpMode, int startValid)
{
    if (!initialized) {
        printf("ERROR: Processor not initialized\n");
        return -1;
    }
    if (beam <= 0 || beam > maxBeam) {
        printf("ERROR: beam %d invalid (max: %d)\n", beam, maxBeam);
        return -1;
    }
    if (frameCount <= 0 || frameCount > CUDA_BATCH_MAX) {
        printf("ERROR: frameCount %d invalid (max: %d)\n", frameCount, CUDA_BATCH_MAX);
        return -1;
    }
    if (!poses || !amps || !tofs || !localZs) {
        printf("ERROR: Null input pointer\n");
        return -1;
    }
    if (!vboRegistered) {
        printf("ERROR: VBO not registered\n");
        return -1;
    }
    if (startValid >= vboMaxPoints) {
        return -2;  // cloud full
    }

    int total = beam * frameCount;
    for (int f = 0; f < frameCount; f++) {
        for (int i = 0; i < 6; i++) h_batchPose[f * 6 + i] = (float)poses[f * 6 + i];
        for (int i = 0; i < beam; i++) {
            h_batchAmp[f * beam + i]     = (float)amps[f * beam + i];
            h_batchTof[f * beam + i]     = (float)tofs[f * beam + i];
            h_batchLocalZ[f * beam + i]  = (float)localZs[f * beam + i];
        }
    }

    cudaError_t err;
    err = cudaGraphicsMapResources(1, &cudaVboPoints, 0);
    if (err != cudaSuccess) return -1;
    err = cudaGraphicsMapResources(1, &cudaVboColors, 0);
    if (err != cudaSuccess) {
        cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
        return -1;
    }

    size_t pointSize = 0, colorSize = 0;
    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboPoints, &pointSize, cudaVboPoints);
    if (err != cudaSuccess) goto cleanup;
    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboColors, &colorSize, cudaVboColors);
    if (err != cudaSuccess) goto cleanup;

    err = cudaMemcpy(d_batchPose, h_batchPose, (size_t)frameCount * 6 * sizeof(float),
                     cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_batchAmp, h_batchAmp, (size_t)total * sizeof(float),
                     cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_batchTof, h_batchTof, (size_t)total * sizeof(float),
                     cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_batchLocalZ, h_batchLocalZ, (size_t)total * sizeof(float),
                     cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;

    cudaMemset(d_validCounter, 0, sizeof(int));

    int centerBeam = (beam - 1) / 2;
    int threadsPerBlock = 256;
    int blocksPerGrid = (total + threadsPerBlock - 1) / threadsPerBlock;
    transformPointsVBatch_Kernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_batchPose, d_batchAmp, d_batchTof, d_batchLocalZ,
        beam, frameCount, centerBeam, 0.3f, isAmpMode, startValid, vboMaxPoints,
        d_validCounter, d_colorLUT, lutSize, d_vboPoints, d_vboColors);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        printf("transformPointsVBatch_Kernel failed: %s\n", cudaGetErrorString(err));
    }

cleanup:
    cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
    cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
    d_vboPoints = nullptr;
    d_vboColors = nullptr;
    return (err == cudaSuccess) ? 0 : -1;
}

// ---------------------------------------------------------------------------
// 批量世界坐标路径：worldXYZ/amp/tof 已按帧顺序扁平化，一次同步写入。
// ---------------------------------------------------------------------------
int CUDAPointCloudProcessor::processCloudBatch(
    const float worldXYZ[], const double amps[], const double tofs[],
    int totalCount, int isAmpMode, int startValid)
{
    if (!initialized) {
        printf("ERROR: Processor not initialized\n");
        return -1;
    }
    if (totalCount <= 0 || totalCount > CUDA_BATCH_MAX * 64) {
        printf("ERROR: totalCount %d invalid\n", totalCount);
        return -1;
    }
    if (!worldXYZ || !amps || !tofs) {
        printf("ERROR: Null input pointer\n");
        return -1;
    }
    if (!vboRegistered) {
        printf("ERROR: VBO not registered\n");
        return -1;
    }
    if (startValid >= vboMaxPoints) {
        return -2;  // cloud full
    }

    for (int i = 0; i < totalCount; i++) {
        h_batchWorld[i * 3 + 0] = worldXYZ[i * 3 + 0];
        h_batchWorld[i * 3 + 1] = worldXYZ[i * 3 + 1];
        h_batchWorld[i * 3 + 2] = worldXYZ[i * 3 + 2];
        h_batchAmp[i] = (float)amps[i];
        h_batchTof[i] = (float)tofs[i];
    }

    cudaError_t err;
    err = cudaGraphicsMapResources(1, &cudaVboPoints, 0);
    if (err != cudaSuccess) return -1;
    err = cudaGraphicsMapResources(1, &cudaVboColors, 0);
    if (err != cudaSuccess) {
        cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
        return -1;
    }

    size_t pointSize = 0, colorSize = 0;
    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboPoints, &pointSize, cudaVboPoints);
    if (err != cudaSuccess) goto cleanup;
    err = cudaGraphicsResourceGetMappedPointer((void**)&d_vboColors, &colorSize, cudaVboColors);
    if (err != cudaSuccess) goto cleanup;

    err = cudaMemcpy(d_batchWorld, h_batchWorld, (size_t)totalCount * 3 * sizeof(float),
                     cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_batchAmp, h_batchAmp, (size_t)totalCount * sizeof(float),
                     cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;
    err = cudaMemcpy(d_batchTof, h_batchTof, (size_t)totalCount * sizeof(float),
                     cudaMemcpyHostToDevice);
    if (err != cudaSuccess) goto cleanup;

    cudaMemset(d_validCounter, 0, sizeof(int));

    int threadsPerBlock = 256;
    int blocksPerGrid = (totalCount + threadsPerBlock - 1) / threadsPerBlock;
    writeCloudPointsKernel<<<blocksPerGrid, threadsPerBlock>>>(
        d_batchWorld, d_batchAmp, d_batchTof, totalCount, isAmpMode,
        startValid, vboMaxPoints, d_validCounter, d_colorLUT, lutSize,
        d_vboPoints, d_vboColors);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        printf("writeCloudPointsKernel(batch) failed: %s\n", cudaGetErrorString(err));
    }

cleanup:
    cudaGraphicsUnmapResources(1, &cudaVboPoints, 0);
    cudaGraphicsUnmapResources(1, &cudaVboColors, 0);
    d_vboPoints = nullptr;
    d_vboColors = nullptr;
    return (err == cudaSuccess) ? 0 : -1;
}

// C interface
// ---------------------------------------------------------------------------
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

int resetVBO(CUDAProcessorHandle processor)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->resetVBO();
}

int setColorLUT(CUDAProcessorHandle processor,
                const unsigned int* lut,
                int size)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->setColorLUT(lut, size);
}

int recolorVBO(CUDAProcessorHandle processor,
               const float* values,
               int count)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->recolorVBO(values, count);
}

int processDirectVBO(CUDAProcessorHandle processor,
                     const double* pose,
                     const double* amp,
                     const double* tof,
                     const double* localZ,
                     int beam,
                     int isAmpMode,
                     int startValid)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->processDirect(
        pose, amp, tof, localZ, beam, isAmpMode, startValid);
}

int processDirectCloudVBO(CUDAProcessorHandle processor,
                          const float* worldXYZ,
                          const double* amp,
                          const double* tof,
                          int count,
                          int isAmpMode,
                          int startValid)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->processCloud(
        worldXYZ, amp, tof, count, isAmpMode, startValid);
}

int processDirectVBatch(CUDAProcessorHandle processor,
                        const double* poses,
                        const double* amps,
                        const double* tofs,
                        const double* localZs,
                        int beam,
                        int frameCount,
                        int isAmpMode,
                        int startValid)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->processDirectBatch(
        poses, amps, tofs, localZs, beam, frameCount, isAmpMode, startValid);
}

int processDirectCloudVBatch(CUDAProcessorHandle processor,
                             const float* worldXYZ,
                             const double* amps,
                             const double* tofs,
                             int totalCount,
                             int isAmpMode,
                             int startValid)
{
    if (!processor) return -1;
    return ((CUDAPointCloudProcessor*)processor)->processCloudBatch(
        worldXYZ, amps, tofs, totalCount, isAmpMode, startValid);
}

int isCUDAAvailable(void)
{
    int deviceCount = 0;
    return (cudaGetDeviceCount(&deviceCount) == cudaSuccess && deviceCount > 0) ? 1 : 0;
}

} // extern "C"
