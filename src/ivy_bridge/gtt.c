#include "gtt.h"

#include <lil/imports.h>

void lil_ivb_vmem_clear(LilGpu* gpu) {
    for (size_t i = 0; i < (gpu->gtt_size >> 12); i++) {
        volatile uint32_t* gtt = (uint32_t*)(gpu->gtt_address + i * 4);
        *gtt = 0;
    }
}

void lil_ivb_vmem_map(LilGpu* gpu, uint64_t host, GpuAddr gpu_addr) {
    if ((host & ~0xFFFFFFFFFF) != 0)
        lil_panic("Ivy Bridge GPU only supports 40-bit host addresses");

    *(volatile uint32_t*)(gpu->gtt_address + (gpu_addr / 0x1000) * 4) = host | (((host >> 32) & 0xFF) << 4) | 0b110 | 1; // MLC/LLC Caching, Present
}
