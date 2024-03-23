#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>

#include <lil/intel.h>

#include "src/helpers.h"
#include "src/regs.h"
#include "src/kaby_lake/pcode.hpp"
#include "src/kaby_lake/watermarks.hpp"

namespace {

void dump_memory_latency(LilGpu *gpu) {
	uint32_t data0 = 0;
	uint32_t data1 = 0;
	uint32_t timeout = 100;
	if(kbl::pcode::rw(gpu, &data0, &data1, kbl::pcode::Mailbox::GEN9_READ_MEM_LATENCY, &timeout)) {
		gpu->mem_latency_first_set = data0;

		data0 = 1;
		data1 = 0;
		timeout = 100;

		if(kbl::pcode::rw(gpu, &data0, &data1, kbl::pcode::Mailbox::GEN9_READ_MEM_LATENCY, &timeout)) {
			gpu->mem_latency_second_set = data0;
		}
	}

	printf("memory latencies:\nlevel 0: %u us\nlevel 1: %u us\nlevel 2: %u us\nlevel 3: %u us\n"
			"level 4: %u us\nlevel 5: %u us\nlevel 6: %u us\nlevel 7: %u us\n",
		(gpu->mem_latency_first_set >> 0) & 0xFF,
		(gpu->mem_latency_first_set >> 8) & 0xFF,
		(gpu->mem_latency_first_set >> 16) & 0xFF,
		(gpu->mem_latency_first_set >> 24) & 0xFF,
		(gpu->mem_latency_second_set >> 0) & 0xFF,
		(gpu->mem_latency_second_set >> 8) & 0xFF,
		(gpu->mem_latency_second_set >> 16) & 0xFF,
		(gpu->mem_latency_second_set >> 24) & 0xFF
	);
}

}

int main() {
	int fd = open("/sys/bus/pci/devices/0000:00:02.0/resource0", O_RDWR | O_CLOEXEC);
	assert(fd > 0);

	void *gpu_mmio = mmap(NULL, 2 * 1024 * 1024, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

	if(gpu_mmio == MAP_FAILED) {
		printf("%s\n", strerror(errno));
		exit(1);
	}

	LilGpu gpu {
		.mmio_start = uintptr_t(gpu_mmio),
	};

	dump_memory_latency(&gpu);

	uint8_t wm[8];
	wm[0] = (gpu.mem_latency_first_set >> 0) & 0xFF;
	wm[1] = (gpu.mem_latency_first_set >> 8) & 0xFF;
	wm[2] = (gpu.mem_latency_first_set >> 16) & 0xFF;
	wm[3] = (gpu.mem_latency_first_set >> 24) & 0xFF;
	wm[4] = (gpu.mem_latency_second_set >> 0) & 0xFF;
	wm[5] = (gpu.mem_latency_second_set >> 8) & 0xFF;
	wm[6] = (gpu.mem_latency_second_set >> 16) & 0xFF;
	wm[7] = (gpu.mem_latency_second_set >> 24) & 0xFF;

	size_t valid_levels = 8;

	for(size_t level = 1; level < valid_levels; level++) {
		if(wm[level] == 0) {
			for(size_t i = level + 1; i < valid_levels; i++) {
				wm[i] = 0;
			}

			valid_levels = level;
			break;
		}
	}

	if(!wm[0]) {
		for(size_t level = 0; level < valid_levels; level++) {
			wm[level] += 2;
		}
	}

	size_t pixel_clock = 138700;
	size_t htotal = 2080;
	printf("linetime %u us\n", div_u32_ceil(htotal * 1000, pixel_clock));

	for(size_t level = 0; level < valid_levels; level++) {
		auto [blocks, lines] = kbl::watermarks::calculate(pixel_clock, 1920, htotal, wm[level], level).value();
		printf("level %zu: %zu blocks %zu lines (level %u)\n", level, blocks, lines, wm[level]);
	}

	pixel_clock = 241500;
	htotal = 2720;
	printf("linetime %u us\n", div_u32_ceil(htotal * 1000, pixel_clock));

	return 0;
}
