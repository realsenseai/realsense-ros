#pragma once
#include <cuda_runtime.h>
#include <cstdint>

// Nearest-neighbour downscale of an interleaved rgb8 image, GPU -> GPU.
// The point is to shrink the frame *before* anything crosses to the host: a 480x270 thumbnail is
// 57 KB against the 6.22 MB of a 1080p frame, so a viewer can look at the real GPU pixels without
// putting a full-frame device->host copy back into the pipeline.
cudaError_t launch_downscale_rgb8(
    const void * src, uint32_t src_w, uint32_t src_h,
    void * dst, uint32_t dst_w, uint32_t dst_h, cudaStream_t stream);
