#pragma once

#include <lil/intel.h>

void lil_ivb_vmem_clear(LilGpu* gpu);
void lil_ivb_vmem_map(LilGpu* gpu, uint64_t host, GpuAddr gpu_addr);
