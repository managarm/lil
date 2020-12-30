#include "intel.h"
#include "gtt.h"

void lil_vmem_map(LilGpu* gpu, uint32_t host, GpuAddr gpu_addr, uint32_t flags) {
   *(volatile uint32_t*)(gpu->gtt_address + (gpu_addr / 0x1000) * 4) = host | 1 | flags;
}

void lil_vmem_clear(LilGpu* gpu) {
    for (size_t i = 0; i < (gpu->gtt_size >> 12); i++) {
        volatile uint32_t* gtt = (uint32_t*)(gpu->gtt_address + i * 4);
        *gtt = 0;
    }
}
