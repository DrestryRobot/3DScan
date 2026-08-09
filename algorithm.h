// algorithm.h
#ifndef ALGORITHM_H
#define ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void* CUDAProcessorHandle;

// 创建/销毁处理器
CUDAProcessorHandle createCUDAProcessor(int maxBeamSize);
void destroyCUDAProcessor(CUDAProcessorHandle processor);

// ============ VBO 直写接口 ============
// 注册 OpenGL VBO 到 CUDA
int registerVBO(CUDAProcessorHandle processor,
                unsigned int vboPoints,
                unsigned int vboColors,
                int maxPoints);

// 注销 VBO
int unregisterVBO(CUDAProcessorHandle processor);

// 直接处理并写入 VBO (零拷贝)
int processDirectVBO(CUDAProcessorHandle processor,
                     const double* pose,
                     const double* amp,
                     const double* tof,
                     const double* localZ,
                     int beam,
                     int isAmpMode,
                     int startValid);

// 方案2：直接写入预计算的世界坐标点（均匀网格）
int processDirectCloudVBO(CUDAProcessorHandle processor,
                          const float* worldXYZ,
                          const double* amp,
                          const double* tof,
                          int count,
                          int isAmpMode,
                          int startValid);

// Reset accumulated VBO point cloud
int resetVBO(CUDAProcessorHandle processor);

// Set the 256-entry color LUT used by the kernel
int setColorLUT(CUDAProcessorHandle processor,
                const unsigned int* lut,
                int size);

// Re-color the accumulated point cloud in the VBO (AMP or TOF mode)
int recolorVBO(CUDAProcessorHandle processor,
               const float* values,
               int count);

// 检查 CUDA 可用性
int isCUDAAvailable(void);

#ifdef __cplusplus
}
#endif

#endif // ALGORITHM_H