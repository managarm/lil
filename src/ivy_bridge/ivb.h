#pragma once

#include <lil/intel.h>

#ifdef __cplusplus
extern "C" {
#endif

void lil_init_ivb_gpu(LilGpu* ret, void* device);

#ifdef __cplusplus
}
#endif
