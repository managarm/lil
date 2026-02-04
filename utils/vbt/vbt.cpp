#include <algorithm>
#include <vector>

#include <lil/imports.h>
#include <lil/vbt.h>

#include "src/vbt/vbt.hpp"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <map>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

static struct option longopts[] = {
	{"blocks", no_argument, 0, 'B'},
	{"block", required_argument, 0, 'b'},
	{"help", no_argument, 0, 'h'},
	{"all", no_argument, 0, 'a'},
	{0, 0, 0, 0},
};

static void dump_block(const struct vbt_header *hdr, int block);

int main(int argc, char *argv[]) {
	opterr = 0;

	bool print_blocks = false;
	bool dump_all_blocks = false;
	std::vector<int> dump_block_ids = {};

	while(1) {
		int option_index = 0;
		int c = getopt_long(argc, argv, "b:Bah", longopts, &option_index);

		if(c == -1) {
			break;
		}

		switch(c) {
			case 'B': {
				print_blocks = true;
				break;
			}
			case 'b': {
				dump_block_ids.push_back(atoi(optarg));
				break;
			}
			case 'a': {
				dump_all_blocks = true;
				break;
			}
			case 'h': {
				printf("help: vbt [-B] [-b BLOCK] [-a] [-] input...");
				exit(0);
			}
			case '?': {
				fprintf(stderr, "error: unknown option '-%c'\n", optopt);
				exit(1);
			}
			default: {
				fprintf(stderr, "error: getopt_long returned '%c'\n", c);
				break;
			}
		}
	}

	if((optind + 1) != argc) {
		fprintf(stderr, "no file given\n");
		exit(1);
	}

	const char *filepath = argv[optind];
	int fd = open(filepath, O_RDONLY);
	if(fd < 0) {
		fprintf(stderr, "could not open file '%s': %s\n", argv[1], strerror(errno));
		exit(1);
	}

	struct stat statbuf;
	int err = fstat(fd, &statbuf);
	assert(err >= 0);

	size_t vbt_len = 0;
	void *vbt;

	if(statbuf.st_size) {
		vbt = mmap(NULL, statbuf.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
		assert(vbt != MAP_FAILED);
		close(fd);
		vbt_len = statbuf.st_size;
	} else {
		int ret;
		size_t len = 0;
		size_t size = 0x2000;
		vbt = malloc(size);

		while((ret = read(fd, reinterpret_cast<void *>(uintptr_t(vbt) + len), size - len))) {
			if(ret < 0) {
				fprintf(stderr, "Failed to read \"%s\": %s\n", filepath, strerror(errno));
				return EXIT_FAILURE;
			}

			len += ret;
			if(len == size) {
				size *= 2;
				vbt = realloc(vbt, size);
			}
		}

		vbt_len = len;
	}

	const struct vbt_header *hdr = vbt_get_header(vbt, vbt_len);
	assert(hdr);

	printf("VBT header version %u.%u\n", hdr->version / 100, hdr->version % 100);

	const struct bdb_header *bdb_hdr = vbt_get_bdb_header(hdr);
	assert(bdb_hdr);

	if(dump_all_blocks) {
		for(size_t i = 0; i < 256; i++) {
			if(vbt_get_bdb_block_generic(hdr, bdb_block_id(i))) {
				dump_block_ids.push_back(i);
			}
		}
	}

	if(print_blocks) {
		printf("Blocks:");
		for(size_t i = 0; i < 256; i++) {
			if(vbt_get_bdb_block_generic(hdr, bdb_block_id(i))) {
				printf(" %zu", i);
			}
		}
		printf("\n");
	}

	std::sort(dump_block_ids.begin(), dump_block_ids.end());

	for(auto id : dump_block_ids) {
		dump_block(hdr, id);
	}

	munmap(vbt, vbt_len);

	return 0;
}

static std::map<size_t, const char *> child_device_type_bits = {
	{DEVICE_TYPE_CLASS_EXTENSION, "Class extension"},
	{DEVICE_TYPE_POWER_MANAGEMENT, "Power management"},
	{DEVICE_TYPE_HOTPLUG_SIGNALING, "Hotplug signaling"},
	{DEVICE_TYPE_INTERNAL_CONNECTOR, "Internal connector"},
	{DEVICE_TYPE_NOT_HDMI_OUTPUT, "Not HDMI output"},
	{DEVICE_TYPE_MIPI_OUTPUT, "MIPI output"},
	{DEVICE_TYPE_COMPOSITE_OUTPUT, "Composite output"},
	{DEVICE_TYPE_DUAL_CHANNEL, "Dual channel"},
	{DEVICE_TYPE_CONTENT_PROTECTION, "Content protection"},
	{DEVICE_TYPE_HIGH_SPEED_LINK, "High speed link"},
	{DEVICE_TYPE_LVDS_SIGNALING, "LVDS signaling"},
	{DEVICE_TYPE_TMDS_DVI_SIGNALING, "TMDS DVI signaling"},
	{DEVICE_TYPE_VIDEO_SIGNALING, "Video signaling"},
	{DEVICE_TYPE_DISPLAYPORT_OUTPUT, "DisplayPort output"},
	{DEVICE_TYPE_DIGITAL_OUTPUT, "Digital output"},
	{DEVICE_TYPE_ANALOG_OUTPUT, "Analog output"},
};

static const char *aux_channel(uint8_t aux_ch) {
	switch(aux_ch) {
		case 0x00:
			return "n/a";
		case 0x40:
			return "AUX channel A";
		case 0x10:
			return "AUX channel B";
		case 0x20:
			return "AUX channel C";
		default:
			return "invalid";
	}
}

static std::map<size_t, const char *> device_handles = {
	{DEVICE_HANDLE_CRT, "CRT"},
	{DEVICE_HANDLE_EFP1, "EFP 1 (HDMI/DVI/DP)"},
	{DEVICE_HANDLE_EFP2, "EFP 2 (HDMI/DVI/DP)"},
	{DEVICE_HANDLE_EFP3, "EFP 3 (HDMI/DVI/DP)"},
	{DEVICE_HANDLE_LFP1, "LFP 1 (eDP)"},
	{DEVICE_HANDLE_LFP2, "LFP 2 (eDP)"},
};

static const char *device_handle(uint8_t handle) {
	for(auto [i, str] : device_handles) {
		if(handle == i) {
			return str;
		}
	}

	return "<invalid>";
}

static const char *i2c_speed(uint8_t speed) {
	switch(speed) {
		case 0:
			return "100 kHz";
		case 1:
			return "50 kHz";
		case 2:
			return "400 kHz";
		case 3:
			return "1 MHz";
		default:
			return "<invalid>";
	}
}

static const char *parse_prd_pipe(uint8_t pipe) {
	if(pipe == 0x04)
		return "Port A";
	if(pipe == 0x40)
		return "Port B";
	if(pipe == 0x20)
		return "Port C";
	if(pipe == 0x10)
		return "Port D";
	if(pipe == 8)
		return "Port EDP";
	return "<invalid port>";
}

static void dump_block(const struct vbt_header *hdr, int block) {
	const struct bdb_block_header *b = vbt_get_bdb_block_generic(hdr, bdb_block_id(block));
	ptrdiff_t offset = (b) ? ((uintptr_t) b - (uintptr_t) hdr) : 0;
	printf("Dumping Block %d (offset 0x%lx):\n", block, offset);

	if(!b) {
		puts("\t<block not present>");
		return;
	}

	switch(block) {
		case BDB_GENERAL_DEFINITIONS: {
			const struct bdb_general_definitions *d = reinterpret_cast<const bdb_general_definitions *>(b);

			size_t children = (d->size - sizeof(*d) + sizeof(struct bdb_block_header)) / d->child_dev_size;
			printf("\tChild dev count %zu\n", children);
			printf("\tChild dev size %u\n", d->child_dev_size);

			for(size_t child = 0; child < children; child++) {
				struct child_device *dev = reinterpret_cast<struct child_device *>((uintptr_t) &d->child_dev + (child * d->child_dev_size));

				if(!dev->device_type)
					continue;

				printf("\tChild %zu:\n", child);
				printf("\t\tDevice handle: %s (0x%04x)\n", device_handle(dev->handle), dev->handle);
				printf("\t\tDevice type: 0x%04x\n", dev->device_type);

				for(size_t bit = 0; bit < 16; bit++) {
					if(dev->device_type & (1 << bit)) {
						printf("\t\t\t%s\n", child_device_type_bits[bit]);
					}
				}

				printf("\t\tAUX channel: %s (0x%02x)\n", aux_channel(dev->aux_channel), dev->aux_channel);
				printf("\t\tDDC pin: 0x%02x\n", dev->ddc_pin);
				printf("\t\tI2C speed: %s (0x%02x)\n", i2c_speed(dev->i2c_speed), dev->i2c_speed);
			}
			break;
		}
		case BDB_OEM_MODE: {
			const struct bdb_oem_mode *oem = reinterpret_cast<const bdb_oem_mode *>(b);

			printf("\tOEM mode entries: %u (max 6)\n", oem->num);

			for(size_t i = 0; i < oem->num; i++) {
				const struct oem_mode_entry *mode = &oem->entries[i];

				if(!mode->x_res && !mode->y_res)
					continue;

				uint8_t bpp = (mode->bpp & 7) ? (8 << __builtin_ctz(mode->bpp & 7)) : 0;
				printf("\t\t[%02zu] %ux%u %u bpp @ %u Hz\n", i, mode->x_res, mode->y_res, bpp, mode->refresh);

				printf("\t\t[%02zu] mode flags:", i);
				if(mode->mode_flags & 7) {
					if(mode->mode_flags & 1)
						printf(" VBIOS");
					if(mode->mode_flags & 2)
						printf(" Driver");
					if(mode->mode_flags & 4)
						printf(" GOP");
				} else {
					printf(" <none>");
				}
				printf("\n");
			}
			break;
		}
		case BDB_FIXED_MODE_SET: {
			const struct bdb_fixed_mode_set *f = reinterpret_cast<const bdb_fixed_mode_set *>(b);
			if(f->feature_enable) {
				printf("\tfixed mode set: %ux%u\n", f->x_res, f->y_res);
			} else {
				printf("\tfixed mode set: disabled\n");
			}
			break;
		}
		case BDB_PRD_BOOT_TABLE: {
			const struct bdb_prd_boot_table *prd = reinterpret_cast<const bdb_prd_boot_table *>(b);

			printf("\tPRD entries: %u (max 16)\n", prd->num);

			for(size_t i = 0; i < prd->num; i++) {
				const struct prd_entry *e = &prd->entries[i];

				if(!e->attach_bits && !e->pipe_a && !e->pipe_b)
					continue;

				printf("\t\tentry %zu\n", i);
				printf("\t\t\tattach bits 0x%02x\n", e->attach_bits);
				printf("\t\t\tpipe A is mapped to %s (0x%02x)\n", parse_prd_pipe(e->pipe_a), e->pipe_a);
				printf("\t\t\tpipe B is mapped to %s (0x%02x)\n", parse_prd_pipe(e->pipe_b), e->pipe_b);
			}
			break;
		}
		default: {
			puts("\t<decoding unavailable>");
			break;
		}
	}

	printf("\n");
}
