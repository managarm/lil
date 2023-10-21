#include <lil/imports.h>

#include "src/helpers.h"
#include "src/coffee_lake/gtt.h"

#define GTT_HAW 39

void lil_cfl_vmem_clear(LilGpu* gpu) {
    for (size_t i = 0; i < (gpu->gtt_size >> 12); i++) {
        GTT64_ENTRY(gpu, i << 12) = 0;
    }
}

void lil_cfl_vmem_map(LilGpu* gpu, uint64_t host, GpuAddr gpu_addr) {
    // TODO: Servers support 46-bits (At least according to the Skylake PRMs); support this
    uint64_t mask = ((1UL << GTT_HAW) - 1) & ~0xFFF;

    if ((host & ~mask) != 0)
        lil_panic("Coffee Lake GPUs only support " STRINGIFY(GTT_HAW) "-bit host addresses");

    GTT64_ENTRY(gpu, gpu_addr) = (host & mask) | GTT_PAGE_PRESENT;
}
