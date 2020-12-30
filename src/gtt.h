#pragma once

#include "intel.h"

//zero out the entire GTT
void lil_vmem_clear(LilGpu* gpu);
void lil_vmem_map(LilGpu* gpu, uint32_t host, GpuAddr gpu_addr, uint32_t flags);
