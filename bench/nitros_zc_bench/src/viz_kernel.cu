#include "viz_kernel.h"

namespace
{
__global__ void downscale_rgb8_kernel(
    const unsigned char * src, uint32_t src_w, uint32_t src_h,
    unsigned char * dst, uint32_t dst_w, uint32_t dst_h)
{
    const uint32_t x = blockIdx.x * blockDim.x + threadIdx.x;
    const uint32_t y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) {
        return;
    }
    // Map the destination pixel back to the nearest source pixel.
    const uint32_t sx = static_cast<uint32_t>((static_cast<uint64_t>(x) * src_w) / dst_w);
    const uint32_t sy = static_cast<uint32_t>((static_cast<uint64_t>(y) * src_h) / dst_h);
    const size_t s = (static_cast<size_t>(sy) * src_w + sx) * 3;
    const size_t d = (static_cast<size_t>(y) * dst_w + x) * 3;
    dst[d + 0] = src[s + 0];
    dst[d + 1] = src[s + 1];
    dst[d + 2] = src[s + 2];
}
}  // namespace

cudaError_t launch_downscale_rgb8(
    const void * src, uint32_t src_w, uint32_t src_h,
    void * dst, uint32_t dst_w, uint32_t dst_h, cudaStream_t stream)
{
    const dim3 block(16, 16);
    const dim3 grid((dst_w + block.x - 1) / block.x, (dst_h + block.y - 1) / block.y);
    downscale_rgb8_kernel<<<grid, block, 0, stream>>>(
        static_cast<const unsigned char *>(src), src_w, src_h,
        static_cast<unsigned char *>(dst), dst_w, dst_h);
    return cudaGetLastError();
}
