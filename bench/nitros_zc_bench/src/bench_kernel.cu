// The GPU work both consumers perform, identical on either path: read every byte of the frame.
#include "bench_kernel.h"

#include <cuda_runtime.h>

namespace
{
__global__ void checksum_kernel(const unsigned char * data, size_t n, unsigned long long * out)
{
    unsigned long long local = 0;
    const size_t stride = static_cast<size_t>(blockDim.x) * gridDim.x;
    for (size_t i = static_cast<size_t>(blockIdx.x) * blockDim.x + threadIdx.x; i < n; i += stride) {
        local += data[i];
    }
    atomicAdd(out, local);
}
}  // namespace

cudaError_t launch_checksum(
    const void * gpu_data, size_t n, unsigned long long * d_out, cudaStream_t stream)
{
    cudaError_t err = cudaMemsetAsync(d_out, 0, sizeof(unsigned long long), stream);
    if (err != cudaSuccess) {
        return err;
    }
    checksum_kernel<<<512, 256, 0, stream>>>(
        static_cast<const unsigned char *>(gpu_data), n, d_out);
    return cudaGetLastError();
}
