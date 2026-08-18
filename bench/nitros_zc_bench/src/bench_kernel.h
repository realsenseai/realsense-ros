#pragma once
#include <cuda_runtime.h>
#include <cstddef>

// Sums every byte of the frame into *d_out on the given stream. Nothing about the result matters;
// it exists so the read of all pixels cannot be optimized away.
cudaError_t launch_checksum(
    const void * gpu_data, size_t n, unsigned long long * d_out, cudaStream_t stream);
