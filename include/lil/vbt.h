#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <lil/intel.h>
#include <lil/vbt-types.h>

/**
 * Obtain a valid VBT header
 *
 * @param vbt Pointer to a VBT in memory
 * @param size The size of the VBT in bytes
 */
const struct vbt_header *vbt_get_header(const void *vbt, size_t size);
const struct bdb_header *vbt_get_bdb_header(const struct vbt_header *hdr);
const struct bdb_block_header *vbt_get_bdb_block(const struct vbt_header *hdr, enum bdb_block_id id);

void vbt_init(LilGpu *gpu);

uint8_t vbt_dvo_to_ddi(uint8_t dvo_id);
const char *vbt_dvo_get_name(uint8_t dvo);

const struct vbt_header *vbt_locate(LilGpu *gpu);
