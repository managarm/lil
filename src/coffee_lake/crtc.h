#pragma once

#include "../intel.h"

void lil_cfl_shutdown (struct LilGpu* gpu, struct LilCrtc* crtc);
void lil_cfl_commit_modeset (struct LilGpu* gpu, struct LilCrtc* crtc);