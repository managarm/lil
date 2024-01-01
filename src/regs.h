#pragma once

#include <lil/imports.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REG_PTR(r) ((volatile uint32_t *) (gpu->mmio_start + (r)))
#define REG(r) (*REG_PTR(r))

#define BLC_PWM_DATA 0x48254

#define GMBUS0 0xC5100
#define GMBUS1 0xC5104
#define GMBUS2 0xC5108
#define GMBUS3 0xC510C

#define GEN6_PCODE_MAILBOX 0x138124
#define GEN6_PCODE_MAILBOX_READY (1 << 31)

#define GEN6_PCODE_DATA 0x138128
#define GEN6_PCODE_DATA1 0x13812C

static inline bool wait_for_bit_set(volatile uint32_t *reg, uint32_t mask, size_t timeout_us, size_t step_us) {
	if(timeout_us % step_us != 0) {
		lil_panic("timeout is not a multiple of step!");
	}

	while((*reg & mask) != mask) {
		lil_usleep(step_us);

		timeout_us -= step_us;

		if(!timeout_us)
			return false;
	}

	return true;
}

static inline bool wait_for_bit_unset(volatile uint32_t *reg, uint32_t mask, size_t timeout_us, size_t step_us) {
	if(timeout_us % step_us != 0) {
		lil_panic("timeout is not a multiple of step!");
	}

	while((*reg & mask) != 0) {
		lil_usleep(step_us);

		timeout_us -= step_us;

		if(!timeout_us)
			return false;
	}

	return true;
}
