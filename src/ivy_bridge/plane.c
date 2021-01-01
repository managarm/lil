#include "plane.h"

#define  PRI_LINOFF_BASE 0x70184
#define  PRI_SURF_BASE 0x7019C

void lil_ivb_update_primary_surface(struct LilGpu* gpu, struct LilPlane* plane, GpuAddr surface_address) {
    volatile uint32_t* pri_surf = (uint32_t*)(gpu->mmio_start + PRI_SURF_BASE + 0x1000 * plane->pipe_id);
    volatile uint32_t* pri_linoff = (uint32_t*)(gpu->mmio_start + PRI_LINOFF_BASE + 0x1000 * plane->pipe_id);

    gpu->connectors[plane->pipe_id].crtc->planes[0].surface_address = surface_address;

    *pri_surf   = surface_address  & ~0xfff;
    *pri_linoff =  surface_address &  0xfff;
}
