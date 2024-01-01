#pragma once

#include <lil/imports.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define REG_PTR(r) ((volatile uint32_t *) (gpu->mmio_start + (r)))
#define REG(r) (*REG_PTR(r))

#define GDT_CHICKEN_BITS 0x09840

#define FUSE_STATUS 0x42000
#define FUSE_STATUS_PG0 (1 << 27)
#define FUSE_STATUS_PG1 (1 << 26)
#define FUSE_STATUS_PG2 (1 << 25)

#define DBUF_CTL 0x45008
#define DBUF_CTL_POWER_ENABLE (1 << 31)
#define DBUF_CTL_POWER_STATE (1 << 30)

#define HSW_PWR_WELL_CTL1 0x45400
#define HSW_PWR_WELL_CTL1_POWER_WELL_2_REQUEST (1 << 31)
#define HSW_PWR_WELL_CTL1_POWER_WELL_2_STATE (1 << 30)
#define HSW_PWR_WELL_CTL1_POWER_WELL_1_REQUEST (1 << 29)
#define HSW_PWR_WELL_CTL1_POWER_WELL_1_STATE (1 << 28)
#define HSW_PWR_WELL_CTL1_POWER_WELL_MISC_IO_REQUEST (1 << 1)
#define HSW_PWR_WELL_CTL1_POWER_WELL_MISC_IO_STATE (1 << 0)

#define CDCLK_CTL 0x46000
#define CDCLK_CTL_DECIMAL_MASK (0x7FF)
#define CDCLK_CTL_DECIMAL(v) (v & CDCLK_CTL_DECIMAL_MASK)
#define CDCLK_CTL_FREQ_SELECT_MASK (3 << 26)
#define CDCLK_CTL_FREQ_SELECT(v) (((v) << 26) & CDCLK_CTL_FREQ_SELECT_MASK)

#define LCPLL1_CTL 0x46010
#define LCPLL1_ENABLE (1u << 31)
#define LCPLL1_LOCK (1 << 30)

#define NDE_RSTWRN_OPT 0x46408

#define BLC_PWM_DATA 0x48254

#define TRANSCODER_A_BASE 0x60000
#define TRANSCODER_B_BASE 0x61000
#define TRANSCODER_C_BASE 0x62000
#define TRANSCODER_EDP_BASE 0x6F000

#define SRD_CTL 0x800
#define SRD_STATUS 0x840

#define DDI_BUF_CTL(i) (0x64000 + ((i) * 0x100))
#define DDI_BUF_CTL_ENABLE (1 << 31)
#define DDI_BUF_CTL_PORT_REVERSAL (1 << 16)
#define DDI_BUF_CTL_IDLE (1 << 7)
#define DDI_BUF_CTL_DDI_A_4_LANES (1 << 4)
#define DDI_BUF_CTL_DP_PORT_WIDTH_MASK (0b111 << 1)
#define DDI_BUF_CTL_DP_PORT_WIDTH(lanes) ((lanes - 1) << 1)
#define DDI_BUF_CTL_DISPLAY_DETECTED (1 << 0)

#define DDI_AUX_CTL(c) (0x64010 + ((c) * 0x100))
#define DDI_AUX_CTL_BUSY (1u << 31)
#define DDI_AUX_CTL_DONE (1 << 30)
#define DDI_AUX_CTL_IRQ (1 << 29)
#define DDI_AUX_CTL_TIMEOUT (1 << 28)
#define DDI_AUX_CTL_TIMEOUT_VAL_MAX (0x3 << 26)
#define DDI_AUX_CTL_RX_ERR (1 << 25)
#define DDI_AUX_SET_MSG_SIZE(sz) (((sz) & 0x1F) << 20)
#define DDI_AUX_GET_MSG_SIZE(ctl) (((ctl) >> 20) & 0x1F)

#define DDI_AUX_CTL_SKL_FW_SYNC_PULSE(v) (((v) - 1) << 5)
#define DDI_AUX_CTL_SKL_SYNC_PULSE(v) ((v) - 1)

#define DDI_AUX_DATA(c) (0x64014 + ((c) * 0x100))
#define DDI_AUX_I2C_WRITE 0x0
#define DDI_AUX_I2C_READ 0x1
#define DDI_AUX_I2C_MOT (1 << 2) // Middle of Transfer

#define DDI_AUX_NATIVE_WRITE 0x8
#define DDI_AUX_NATIVE_READ 0x9

#define DPLL_CTRL1 0x6C058
#define DPLL_CTRL1_LINK_RATE_MASK(i) (7 << ((i) * 6 + 1))
#define DPLL_CTRL1_LINK_RATE(i, v) ((v) << ((i) * 6 + 1))
#define DPLL_CTRL1_LINK_RATE_2700_MHZ 0b000
#define DPLL_CTRL1_LINK_RATE_1350_MHZ 0b001
#define DPLL_CTRL1_LINK_RATE_810_MHZ 0b010
#define DPLL_CTRL1_LINK_RATE_1620_MHZ 0b011
#define DPLL_CTRL1_LINK_RATE_1080_MHZ 0b100
#define DPLL_CTRL1_LINK_RATE_2160_MHZ 0b101
#define DPLL_CTRL1_PROGRAM_ENABLE(i) (1 << ((i) * 6))
#define DPLL_CTRL1_SSC_ENABLE(i) (1 << ((i) * 6 + 4))
#define DPLL_CTRL1_HDMI_MODE(i) (1 << ((i) * 6 + 5))

#define DPLL_STATUS 0x6C060
#define DPLL_STATUS_LOCK(i) (1 << ((i) * 8))

#define SFUSE_STRAP 0xC2014

#define SOUTH_DSPCLK_GATE_D 0xC2020
#define SOUTH_DSPCLK_GATE_D_PCH_LP_PARTITION_LEVEL_DISABLE 0x1000

#define SHOTPLUG_CTL 0xC4030

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
