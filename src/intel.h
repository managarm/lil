#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

typedef uint32_t GpuAddr;
struct LilGpu;

typedef enum LilConnectorType {
    HDMI,
    LVDS,
    DISPLAYPORT,
    EDP
} LilConnectorType;

typedef struct LilModeInfo {
    uint32_t clock;
    int vactive;
	int vsyncStart;
	int vsyncEnd;
	int vtotal;
	int hactive;
	int hsyncStart;
	int hsyncEnd;
	int htotal;
    int bpc;
} LilModeInfo;

/*
 * Note: intel gpus only have one valid (connector, encoder, crtc)
 */

typedef enum LilPlaneType {
    PRIMARY, CURSOR, SPRITE,
} LilPlaneType;

//generic plane
typedef struct LilPlane {
    LilPlaneType type;
    GpuAddr surface_address;
    bool enabled;
    uint32_t pipe_id;

    bool (*update_surface) (struct LilGpu* gpu, struct LilPlane* plane, GpuAddr surface_address, GpuAddr line_stride);
} LilPlane;

struct LilConnector;

typedef enum LilTranscoder {
    TRANSCODER_A,
    TRANSCODER_B,
    TRANSCODER_C,
    TRANSCODER_D,

    TRANSCODER_EDP,
} LilTranscoder;

typedef struct LilCrtc {
    LilModeInfo current_mode;
    struct LilConnector* connector;

    LilTranscoder transcoder;

    uint32_t pipe_id;

    uint32_t num_planes;
    LilPlane* planes;

    void (*shutdown) (struct LilGpu* gpu, struct LilCrtc* crtc);
    void (*commit_modeset) (struct LilGpu* gpu, struct LilCrtc* crtc);
} LilCrtc;

typedef struct LilConnectorInfo {
    uint32_t num_modes;
    LilModeInfo* modes;
} LilConnectorInfo;

typedef struct LilMinMax {
    int min, max;
} LilMinMax;

typedef struct LilLimit {
    int dot_limit;
    int slow, fast;
} LilLimit;

typedef struct PllLilLimits {
    LilMinMax dot, vco, n, m, m1, m2, p, p1;
    LilLimit p2;
} PllLilLimits;

typedef struct LilConnector {
    bool (*is_connected) (struct LilGpu* gpu, struct LilConnector* connector);
    LilConnectorInfo (*get_connector_info) (struct LilGpu* gpu, struct LilConnector* connector);
    void (*set_state) (struct LilGpu* gpu, struct LilConnector* connector, uint32_t state);
    uint32_t (*get_state) (struct LilGpu* gpu, struct LilConnector* connector);

    uint32_t id;
    LilConnectorType type;

    PllLilLimits limits;

    LilCrtc* crtc;

    bool on_pch;

    //this field is changed by lil_process_interrupt
    //indicates whether vertical sync/blank is happening for this connector.
    bool vsync;
    bool vblank;
} LilConnector;

typedef enum LilInterruptEnableMask {
    VBLANK_A, VSYNC_A,
    VBLANK_B, VSYNC_B,
    VBLANK_C, VSYNC_C
} LilInterruptEnableMask;

typedef enum LilGpuGen {
    GEN_IVB = 7,
    GEN_CFL = 9,
} LilGpuGen;

typedef struct LilGpu {
    uint32_t num_connectors;
    LilConnector* connectors;

    LilGpuGen gen;

    uintptr_t gpio_start;
    uintptr_t mmio_start;
    uintptr_t vram;
    uintptr_t gtt_address;
    size_t gtt_size;

    void (*enable_display_interrupt) (struct LilGpu* gpu, uint32_t enable_mask);
    void (*process_interrupt) (struct LilGpu* gpu);

    void (*vmem_clear) (struct LilGpu* gpu);
    void (*vmem_map) (struct LilGpu* gpu, uint64_t host, GpuAddr gpu_addr);
} LilGpu;

/*
 * Fills out all the structures relating to the gpu
 *
 * The general modesetting procedure consists of the following:
 * - initialize the gpu
 * - set current_mode in the crtc you want to modeset
 * - allocate memory for the planes that are going to be used
 * - set the surface_address for the planes
 * - call commit_modeset
 */
void lil_init_gpu(LilGpu* ret, void* pci_device);

typedef struct LilIrqType {
} LilIrqType;
