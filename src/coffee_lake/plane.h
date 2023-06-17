#pragma once

#include <lil/intel.h>

bool lil_cfl_update_primary_surface(struct LilGpu* gpu, struct LilPlane* plane, GpuAddr surface_address, GpuAddr line_stride);
