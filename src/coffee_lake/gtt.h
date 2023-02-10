#pragma once

#include <lil/intel.h>

void lil_cfl_vmem_clear(LilGpu* gpu);
void lil_cfl_vmem_map(LilGpu* gpu, uint64_t host, GpuAddr gpu_addr);
