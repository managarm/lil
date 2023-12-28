#include <stddef.h>
#include <stdint.h>

#include <lil/imports.h>
#include <lil/vbt.h>

const struct vbt_header *vbt_get_header(const void *vbt, size_t size) {
	for(size_t i = 0; i + 4 < size; i++) {
		const struct vbt_header *header = (vbt + i);

		if(memcmp(header->signature, VBT_HEADER_SIGNATURE, 4))
			continue;

		return header;
	}

	return NULL;
}

const struct bdb_header *vbt_get_bdb_header(const struct vbt_header *hdr) {
	struct bdb_header *bdb = (void *) ((uintptr_t) hdr + hdr->bdb_offset);

	if(memcmp(bdb->signature, BDB_HEADER_SIGNATURE, 16))
		return NULL;

	return bdb;
}

static size_t bdb_block_size(const void *hdr) {
	if(*(uint8_t *)hdr == BDB_MIPI_SEQUENCE && *(uint8_t *)(hdr + 3) >= 3)
		return *((const uint32_t *)(hdr + 4)) + 3;
	else
		return *((const uint16_t *)(hdr + 1)) + 3;
}

const struct bdb_block_header *vbt_get_bdb_block(const struct vbt_header *hdr, enum bdb_block_id id) {
	const struct bdb_header *bdb = vbt_get_bdb_header(hdr);
	const struct bdb_block_header *block = (const void *) ((uintptr_t) bdb + bdb->header_size);
	size_t index = 0;

	for(; index + 3 < bdb->bdb_size && index + bdb_block_size(block) <= bdb->bdb_size; block = (void *) ((uintptr_t) block + bdb_block_size(block))) {
		if(id == block->id)
			return block;
	}

	return NULL;
}

uint8_t vbt_dvo_to_ddi(uint8_t dvo_id) {
	switch(dvo_id) {
		case 1:
			return 1;
		case 2:
			return 2;
		case 3:
			return 3;
		case 7:
			return 1;
		case 8:
			return 2;
		case 9:
			return 3;
	}

	if(dvo_id == 11 || dvo_id == 12)
		return 4;

	return 0;
}

#define DVO_PORT_HDMIA 0
#define DVO_PORT_HDMIB 1
#define DVO_PORT_HDMIC 2
#define DVO_PORT_HDMID 3
#define DVO_PORT_LVDS 4
#define DVO_PORT_TV 5
#define DVO_PORT_CRT 6
#define DVO_PORT_DPB 7
#define DVO_PORT_DPC 8
#define DVO_PORT_DPD 9
#define DVO_PORT_DPA 10
#define DVO_PORT_DPE 11
#define DVO_PORT_HDMIE 12
#define DVO_PORT_DPF 13
#define DVO_PORT_HDMIF 14
#define DVO_PORT_DPG 15
#define DVO_PORT_HDMIG 16
#define DVO_PORT_DPH 17
#define DVO_PORT_HDMIH 18
#define DVO_PORT_DPI 19
#define DVO_PORT_HDMII 20
#define DVO_PORT_MIPIA 21
#define DVO_PORT_MIPIB 22
#define DVO_PORT_MIPIC 23
#define DVO_PORT_MIPID 24
#define DVO_PORT_MAX DVO_PORT_MIPID

static const char *dvo_port_names[] = {
	[DVO_PORT_CRT] = "CRT",
	[DVO_PORT_DPA] = "DP-A",
	[DVO_PORT_DPB] = "DP-B",
	[DVO_PORT_DPC] = "DP-C",
	[DVO_PORT_DPD] = "DP-D",
	[DVO_PORT_DPE] = "DP-E",
	[DVO_PORT_DPF] = "DP-F",
	[DVO_PORT_DPG] = "DP-G",
	[DVO_PORT_DPH] = "DP-H",
	[DVO_PORT_DPI] = "DP-I",
	[DVO_PORT_HDMIA] = "HDMI-A",
	[DVO_PORT_HDMIB] = "HDMI-B",
	[DVO_PORT_HDMIC] = "HDMI-C",
	[DVO_PORT_HDMID] = "HDMI-D",
	[DVO_PORT_HDMIE] = "HDMI-E",
	[DVO_PORT_HDMIF] = "HDMI-F",
	[DVO_PORT_HDMIG] = "HDMI-G",
	[DVO_PORT_HDMIH] = "HDMI-H",
	[DVO_PORT_HDMII] = "HDMI-I",
	[DVO_PORT_LVDS] = "LVDS",
	[DVO_PORT_MIPIA] = "MIPI-A",
	[DVO_PORT_MIPIB] = "MIPI-B",
	[DVO_PORT_MIPIC] = "MIPI-C",
	[DVO_PORT_MIPID] = "MIPI-D",
	[DVO_PORT_TV] = "TV",
};

const char *vbt_dvo_get_name(uint8_t dvo) {
	if(dvo > DVO_PORT_MAX)
		return NULL;

	return dvo_port_names[dvo];
}
