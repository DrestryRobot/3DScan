# 3DScan

A Qt + VTK + CUDA desktop application that renders ultrasonic scanning data as a
3D point cloud, plus CSV replay for offline inspection.

## Overview

`3DScan` is the 3D visualization front end of the SoundScan ultrasonic scanning
system. Scan frames (robot pose, beam index, amplitude and time-of-flight for up
to 64 beams) are converted into world-space points and pushed into OpenGL vertex
buffer objects (VBOs) by a CUDA kernel, so the cloud can be updated every frame
without a round trip through host memory.

- **Zero-copy rendering** — `algorithm.cu` maps the GL VBOs into CUDA, applies
  the robot pose transform on the GPU and writes positions/colors directly (see
  `registerVBO` / `processDirectVBO` in `algorithm.h`).
- **Batched GPU work** — up to `CUDA_BATCH_MAX` (16) frames are accumulated per
  map/kernel/unmap cycle to cut per-frame synchronization cost.
- **CSV replay** — pass a scan CSV on the command line and playback starts
  automatically shortly after the window appears (`scan.cpp`, `scandata.cpp`).
- **Shared data interface with SoundScan** — cross-module scan/robot globals
  (`amp[64]`, `tof[64]`, `beamValid[64]`, `robot_*`, `robot_ipoc`, ...) are
  declared in `scandata.h` and guarded by `g_scanDataMutex`, because the live
  acquisition threads and the offline loader touch the same state.
- **On-screen data panel** — `datapanel.h` overlays status (data/render FPS, scan
  time, point count), latest-frame info (IPOC, SI, beam, pose, AMP[0], TOF[0])
  and measurements on top of the VTK widget.

## Repository layout

| Path | Purpose |
| --- | --- |
| `main.cpp` | Entry point; installs the file logger and handles an optional CSV argument |
| `mainwindow3.{h,cpp,ui}` | Main window, VTK render widget hosting, playback control |
| `scan.{h,cpp}` | Scan pipeline: pose interpolation, uniform grid resampling, playback timing |
| `scandata.{h,cpp}` | Shared scan/robot globals and the `g_scanDataMutex` lock |
| `algorithm.{h,cu}` | CUDA processor: VBO registration, pose transform, point/color generation |
| `vtkvboactor.{h,cpp}` | `vtkActor` subclass drawing the CUDA-filled VBOs, with display-stride decimation |
| `datapanel.{h,cpp}` | Floating status / latest-frame / measurement panel |
| `3DScan.pro` | qmake project, including the custom `nvcc` compiler rules |

## Requirements

- Windows with a CUDA-capable NVIDIA GPU
- Qt 6 (`core gui widgets opengl openglwidgets network`) and the MSVC toolchain
- CUDA Toolkit — paths are hard-coded to `C:\CUDA\v13.3` in `3DScan.pro`
- VTK (`vtkActor`) development files
- Windows SDK for the OpenGL headers

## Build

`3DScan.pro` drives the build through qmake and the MSVC toolchain (Qt Creator,
or `qmake` followed by `nmake`/`jom`). The CUDA source is compiled by a custom
compiler rule that invokes `nvcc` with `--use_fast_math` and `-arch=sm_86`.
Change `CUDA_ARCH` for a different GPU generation and update `CUDA_DIR` /
`CUDA_INC` if CUDA is not installed at `C:\CUDA\v13.3`.

## Running

```
3DScan.exe              # live view
3DScan.exe scan.csv     # open a scan CSV and start replay
```

Debug output is appended to `C:/3dscan/debug.log`, so the directory must be
writable.

## Notes

- Several paths (CUDA directory, log file, scan directories) are absolute and
  Windows-specific and need editing on another machine.
- The data interface is intentionally aligned with the SoundScan project so the
  two applications can share the same scan/robot data structures.

## License

No license file is present in this repository. Contact the repository owner for
licensing terms.
