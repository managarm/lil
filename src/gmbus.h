#pragma once
#include <lil/intel.h>

#include <stdint.h>
#include <stddef.h>

uint8_t lil_gmbus_get_mode_info(LilGpu* gpu, LilModeInfo* out, int pin_pair);
