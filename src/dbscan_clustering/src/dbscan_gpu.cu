#include <cuda_runtime.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include "dbscan_clustering/dbscan_gpu.cuh"

// Simple error check
#define CUDA_CHECK(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
  if (code != cudaSuccess) {
    fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
    if (abort) exit(code);
  }
}

__global__ void neighbor_kernel(const float* xyz, int N, float eps2, int max_neighbors, int* neighbors, int* neighbor_counts)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= N) return;
  const float xi = xyz[3*i + 0];
  const float yi = xyz[3*i + 1];
  const float zi = xyz[3*i + 2];
  int count = 0;
  for (int j = 0; j < N && count < max_neighbors; ++j) {
    const float dx = xi - xyz[3*j + 0];
    const float dy = yi - xyz[3*j + 1];
    const float dz = zi - xyz[3*j + 2];
    const float d2 = dx*dx + dy*dy + dz*dz;
    if (d2 <= eps2) {
      neighbors[i * max_neighbors + count] = j;
      ++count;
    }
  }
  neighbor_counts[i] = count;
  for (int k = count; k < max_neighbors; ++k) neighbors[i * max_neighbors + k] = -1;
}

void dbscan_gpu_query_neighbors(const float* xyz_host, int N, float eps, int max_neighbors, int* neighbors_host, int* neighbor_counts_host)
{
  dbscan_gpu_query_neighbors_timed(
    xyz_host, N, eps, max_neighbors, neighbors_host, neighbor_counts_host, nullptr);
}

void dbscan_gpu_query_neighbors_timed(
  const float * xyz_host,
  int N,
  float eps,
  int max_neighbors,
  int * neighbors_host,
  int * neighbor_counts_host,
  DbscanGpuTiming * timing)
{
  using Clock = std::chrono::steady_clock;

  if (N <= 0) return;
  if (timing != nullptr) {
    *timing = DbscanGpuTiming{};
  }
  size_t xyz_bytes = sizeof(float) * 3 * N;
  size_t neighbors_bytes = sizeof(int) * N * max_neighbors;
  size_t counts_bytes = sizeof(int) * N;

  float* d_xyz = nullptr;
  int* d_neighbors = nullptr;
  int* d_counts = nullptr;

  const auto t_alloc_begin = Clock::now();
  CUDA_CHECK(cudaMalloc((void**)&d_xyz, xyz_bytes));
  CUDA_CHECK(cudaMalloc((void**)&d_neighbors, neighbors_bytes));
  CUDA_CHECK(cudaMalloc((void**)&d_counts, counts_bytes));
  const auto t_alloc_end = Clock::now();

  const auto t_h2d_begin = Clock::now();
  CUDA_CHECK(cudaMemcpy(d_xyz, xyz_host, xyz_bytes, cudaMemcpyHostToDevice));
  const auto t_h2d_end = Clock::now();

  int threads = 256;
  int blocks = (N + threads - 1) / threads;

  cudaEvent_t kernel_start;
  cudaEvent_t kernel_stop;
  CUDA_CHECK(cudaEventCreate(&kernel_start));
  CUDA_CHECK(cudaEventCreate(&kernel_stop));
  CUDA_CHECK(cudaEventRecord(kernel_start));
  neighbor_kernel<<<blocks, threads>>>(d_xyz, N, eps * eps, max_neighbors, d_neighbors, d_counts);
  CUDA_CHECK(cudaEventRecord(kernel_stop));
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
  CUDA_CHECK(cudaEventSynchronize(kernel_stop));

  float kernel_ms = 0.0F;
  CUDA_CHECK(cudaEventElapsedTime(&kernel_ms, kernel_start, kernel_stop));
  CUDA_CHECK(cudaEventDestroy(kernel_start));
  CUDA_CHECK(cudaEventDestroy(kernel_stop));

  const auto t_d2h_begin = Clock::now();
  CUDA_CHECK(cudaMemcpy(neighbors_host, d_neighbors, neighbors_bytes, cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(neighbor_counts_host, d_counts, counts_bytes, cudaMemcpyDeviceToHost));
  const auto t_d2h_end = Clock::now();

  const auto t_free_begin = Clock::now();
  cudaFree(d_xyz);
  cudaFree(d_neighbors);
  cudaFree(d_counts);
  const auto t_free_end = Clock::now();

  if (timing != nullptr) {
    timing->alloc_ms = std::chrono::duration<float, std::milli>(t_alloc_end - t_alloc_begin).count();
    timing->h2d_ms = std::chrono::duration<float, std::milli>(t_h2d_end - t_h2d_begin).count();
    timing->kernel_ms = kernel_ms;
    timing->d2h_ms = std::chrono::duration<float, std::milli>(t_d2h_end - t_d2h_begin).count();
    timing->free_ms = std::chrono::duration<float, std::milli>(t_free_end - t_free_begin).count();
  }
}
