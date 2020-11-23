#pragma once
#include "intel.h"

#include <stdint.h>
#include <stddef.h>

void lil_get_mode_info(LilGpu* gpu, LilModeInfo* out, int pin_pair);
