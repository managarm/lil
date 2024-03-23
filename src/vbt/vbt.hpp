#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <lil/intel.h>
#include <lil/vbt-types.h>

const struct bdb_header *vbt_get_bdb_header(const struct vbt_header *hdr);

inline size_t bdb_block_size(const void *hdr) {
	if(*(uint8_t *)hdr == BDB_MIPI_SEQUENCE && *(uint8_t *)(uintptr_t(hdr) + 3) >= 3)
		return *((const uint32_t *)(uintptr_t(hdr) + 4)) + 3;
	else
		return *((const uint16_t *)(uintptr_t(hdr) + 1)) + 3;
}

template<typename T>
const T *vbt_get_bdb_block(const struct vbt_header *hdr, enum bdb_block_id id) {
	const struct bdb_header *bdb = vbt_get_bdb_header(hdr);
	auto block = reinterpret_cast<const struct bdb_block_header *>(uintptr_t(bdb) + bdb->header_size);
	size_t index = 0;

	for(; index + 3 < bdb->bdb_size && index + bdb_block_size(block) <= bdb->bdb_size;
			block = reinterpret_cast<const struct bdb_block_header *>(uintptr_t(block) + bdb_block_size(block))) {
		if(id == block->id)
			return reinterpret_cast<const T *>(block);
	}

	return NULL;
}

void vbt_init(LilGpu *gpu);
void vbt_setup_children(LilGpu *gpu);

LilDdiId vbt_dvo_to_ddi(uint8_t dvo_id);
const char *vbt_dvo_get_name(uint8_t dvo);
