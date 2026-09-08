#pragma once

struct DbscanGpuTiming
{
  float alloc_ms{0.0F};
  float h2d_ms{0.0F};
  float kernel_ms{0.0F};
  float d2h_ms{0.0F};
  float free_ms{0.0F};
};

extern "C" {
  // xyz: pointer to float array of size 3*N (x,y,z)
  // neighbors: int array of size N * max_neighbors (filled with neighbor indices or -1)
  // neighbor_counts: int array of size N (number of neighbors found, capped by max_neighbors)
  void dbscan_gpu_query_neighbors(const float* xyz, int N, float eps, int max_neighbors, int* neighbors, int* neighbor_counts);
  void dbscan_gpu_query_neighbors_timed(
    const float * xyz,
    int N,
    float eps,
    int max_neighbors,
    int * neighbors,
    int * neighbor_counts,
    DbscanGpuTiming * timing);
}
