#include "gtt.h"

#include <lil/imports.h>

void lil_cfl_vmem_clear(LilGpu* gpu) {
    for (size_t i = 0; i < (gpu->gtt_size >> 12); i++) {
        volatile uint64_t* gtt = (uint64_t*)(gpu->gtt_address + i * 8);
        *gtt = 0;
    }
}

void lil_cfl_vmem_map(LilGpu* gpu, uint64_t host, GpuAddr gpu_addr) {
    if ((host & ~0xFFFFFFFFFF) != 0)
        lil_panic("Coffee Lake GPUs only supports 40-bit host addresses"); // TODO: Servers support 46-bits (At least according to the Skylake PRMs) support this

    *(volatile uint64_t*)(gpu->gtt_address + (gpu_addr / 0x1000) * 8) = host | 1; // Present
}
