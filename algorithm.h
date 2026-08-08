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
                     double si,
                     int beam,
                     int isAmpMode);

// 检查 CUDA 可用性
int isCUDAAvailable(void);

#ifdef __cplusplus
}
#endif

#endif // ALGORITHM_H
