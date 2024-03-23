#include <lil/imports.h>
#include <lil/intel.h>

#include "src/dpcd.hpp"
#include "src/helpers.h"
#include "src/kaby_lake/cdclk.hpp"
#include "src/kaby_lake/crtc.hpp"
#include "src/kaby_lake/ddi.hpp"
#include "src/kaby_lake/dp-aux.hpp"
#include "src/kaby_lake/edp.hpp"
#include "src/kaby_lake/hpd.hpp"
#include "src/kaby_lake/link-training.hpp"
#include "src/kaby_lake/pipe.hpp"
#include "src/kaby_lake/plane.hpp"
#include "src/kaby_lake/pll.hpp"
#include "src/kaby_lake/transcoder.hpp"
#include "src/regs.h"

namespace {

uint8_t dp_link_rate_for_crtc(LilGpu *gpu, struct LilCrtc* crtc) {
	uint8_t last_valid_index = 0;
	uint32_t link_rate = 0;
	uint32_t max_link_rate = 0;
	LilEncoder *enc = crtc->connector->encoder;

	for(uint8_t i = 0; i < enc->edp.supported_link_rates_len; i++) {
		uint16_t entry = enc->edp.supported_link_rates[i];

		if(entry == 8100 || entry == 10800 || (entry != 12150 && (entry == 1350 || entry == 16200 || entry == 21600 || entry == 27000))) {
			link_rate = 8 * enc->edp.edp_lane_count * (1000 * entry / 5) / (10 * enc->edp.edp_color_depth);

			if(link_rate >= crtc->current_mode.clock && link_rate < max_link_rate) {
				last_valid_index = i;
				max_link_rate = link_rate;
			}
		}
	}

	return last_valid_index;
}

void enable_vblank(LilGpu *gpu, LilCrtc *crtc) {
	REG(IMR(crtc->pipe_id)) &= ~1;
}

} // namespace

namespace kbl::dp {

bool is_connected(struct LilGpu* gpu, struct LilConnector* connector) {
	// TODO(): not reliable, only valid on chip startup, also wrong on some generations for some reason
	if(connector->ddi_id == DDI_A) {
		return REG(DDI_BUF_CTL(DDI_A)) & DDI_BUF_CTL_DISPLAY_DETECTED;
	}

	uint32_t sfuse_mask = 0;
	uint32_t sde_isr_mask = 0;

	switch(connector->ddi_id) {
		case DDI_B: {
			sde_isr_mask = (1 << 4);
			sfuse_mask = (1 << 2);
			break;
		}
		case DDI_C: {
			sde_isr_mask = (1 << 5);
			sfuse_mask = (1 << 1);
			break;
		}
		case DDI_D: {
			sfuse_mask = (1 << 0);
			break;
		}
		default: {
			lil_panic("unhandled DDI id");
		}
	}

	if((REG(SFUSE_STRAP) & sfuse_mask) == 0) {
		return false;
	}

	return REG(SDEISR) & sde_isr_mask;
}

LilConnectorInfo get_connector_info(struct LilGpu* gpu, struct LilConnector* connector) {
    (void)connector;
    LilConnectorInfo ret = {};
    LilModeInfo* info = (LilModeInfo *) lil_malloc(sizeof(LilModeInfo) * 4);

    DisplayData edid = {};
    kbl::dp::aux::read_edid(gpu, connector, &edid);

    uint32_t edp_max_pixel_clock = 990 * gpu->boot_cdclk_freq;

    int j = 0;
    for(int i = 0; i < 4; i++) { // Maybe 4 Detailed Timings
        if(edid.detailTimings[i].pixelClock == 0)
            continue; // Not a timing descriptor

        if(connector->type == EDP && edp_max_pixel_clock && (edid.detailTimings[i].pixelClock * 10) > edp_max_pixel_clock) {
            lil_log(WARNING, "EDID: skipping detail timings %u: pixel clock (%u KHz) > max (%u KHz)\n", i, (edid.detailTimings[i].pixelClock * 10), edp_max_pixel_clock);
            continue;
        }

        edid_timing_to_mode(&edid, edid.detailTimings[i], &info[j++]);
    }

    ret.modes = info;
    ret.num_modes = j;
    return ret;
}

void commit_modeset(struct LilGpu* gpu, struct LilCrtc* crtc) {
	LilConnector *con = crtc->connector;
	LilEncoder *enc = con->encoder;

	if(enc->edp.dynamic_cdclk_supported && gpu->connectors[0].type == EDP) {
		kbl::cdclk::set_for_pixel_clock(gpu, &crtc->current_mode.clock);
	} else {
		if(gpu->cdclk_freq != gpu->boot_cdclk_freq) {
			kbl::cdclk::set_freq(gpu, gpu->boot_cdclk_freq);
		}
	}

	if(crtc->current_mode.bpp == -1) {
		if(con->type == EDP) {
			crtc->current_mode.bpp = enc->edp.edp_color_depth;
		} else {
			lil_panic("crtc->current_mode.bpp == -1\n");
		}
	}

	kbl::crtc::pll_find(gpu, crtc);

	REG(SWF_24) = 8;

	uint32_t stride = (crtc->current_mode.hactive * 4 + 63) >> 6;

	REG(PLANE_CTL(crtc->pipe_id)) = (REG(PLANE_CTL(crtc->pipe_id)) & 0xF0FFFFFF) | 0x4000000;

	uint8_t link_rate_index = 0;

	if(con->type == EDP && enc->edp.edp_dpcd_rev >= 3) {
		link_rate_index = dp_link_rate_for_crtc(gpu, crtc);
		uint16_t looked_up_link_rate = enc->edp.supported_link_rates[link_rate_index];

		if(((looked_up_link_rate == 10800 || looked_up_link_rate == 21600) && !gpu->vco_8640) ||
			((looked_up_link_rate != 10800 && looked_up_link_rate != 21600) && gpu->vco_8640)) {
			lil_panic("unimplemented");
		}
	}

	uint32_t lanes = 0;
	uint32_t max_lanes;
	if(con->type == EDP) {
		max_lanes = enc->edp.edp_max_lanes;
	} else {
		lanes = enc->dp.dp_lane_count;
		max_lanes = enc->dp.dp_lane_count;
		lil_log(VERBOSE, "lil_kbl_commit_modeset: dp lanes=%u, max_lanes=%u\n", lanes, max_lanes);
	}

	if(con->type == EDP) {
		if(enc->edp.edp_fast_link_training_supported) {
			lanes = enc->edp.edp_fast_link_lanes;
		} else {
			lanes = enc->edp.edp_lane_count & 0x1F;
		}
	}
	if(lanes < max_lanes)
		max_lanes = lanes;

	uint32_t link_rate_mhz = 0;

	if(con->type == EDP) {
		if(enc->edp.edp_dpcd_rev >= 3) {
			lil_log(VERBOSE, "edp_dpcd_rev >= 3: link_rate_index=%u\n", link_rate_index);
			enc->edp.edp_max_link_rate = link_rate_index;
			link_rate_mhz = 200 * enc->edp.supported_link_rates[link_rate_index];
		} else {
			switch(enc->edp.edp_max_link_rate) {
				case 6:
					link_rate_mhz = 1620000;
					break;
				case 10:
					link_rate_mhz = 2700000;
					break;
				case 20:
					link_rate_mhz = 5400000;
					break;
				default:
					lil_log(VERBOSE, "edp_max_link_rate=%u\n", enc->edp.edp_max_link_rate);
					lil_panic("invalid edp_max_link_rate");
			}
		}
	} else {
		switch(enc->dp.dp_max_link_rate) {
			case 6:
				link_rate_mhz = 1620000;
				break;
			case 10:
				link_rate_mhz = 2700000;
				break;
			case 20:
				link_rate_mhz = 5400000;
				break;
			default:
				lil_log(VERBOSE, "invalid enc->dp.dp_max_link_rate=%u\n", enc->dp.dp_max_link_rate);
				lil_panic("invalid dp_max_link_rate");
		}
	}

	if(!link_rate_mhz)
		lil_panic("invalid link_rate_mhz == 0");

	// TODO(REG;CORRECTNESS)	we should set the lane count in DDI_BUF_CTL_A
	kbl::dp::aux::native_write(gpu, con, LANE_COUNT_SET, lanes);
	// dp_aux_native_write(gpu, con, LINK_RATE_SET, enc->dp.dp_max_link_rate);

	while((REG(PP_STATUS) & 0x38000000) != 0);
	REG(PP_CONTROL) |= 1;
	kbl::edp::aux_readable(gpu, con);
	REG(PP_CONTROL) &= ~8;

	uint32_t link_rate = link_rate_mhz / 10;
	uint32_t bpp = crtc->current_mode.bpp;

	kbl::edp::validate_clocks_for_bpp(gpu, crtc, max_lanes, link_rate, &bpp);

	if(REG(DP_TP_CTL(con->ddi_id)) & DP_TP_CTL_ENABLE) {
		uint32_t dp_tp_ctl_val = (REG(DP_TP_CTL(con->ddi_id)) & 0xFFFFF8FF) | DP_TP_CTL_TRAIN_PATTERN_IDLE;
		REG(DP_TP_CTL(con->ddi_id)) = dp_tp_ctl_val;
		lil_usleep(17000);
		REG(DP_TP_CTL(con->ddi_id)) = dp_tp_ctl_val & ~DP_TP_CTL_ENABLE;
	}

	kbl::pll::dpll_ctrl_enable(gpu, crtc, link_rate);

	if(max_lanes == 4)
		REG(DDI_BUF_CTL(con->ddi_id)) |= DDI_BUF_CTL_DDI_A_4_LANES;

	kbl::pll::dpll_clock_set(gpu, crtc);
	kbl::ddi::power_enable(gpu, crtc);

	uint32_t dp_tp_ctl_flags = 0;

	if(con->type == EDP && enc->edp.edp_lane_count & 0x80)
		dp_tp_ctl_flags = DP_TP_CTL_ENHANCED_FRAMING_ENABLE;

	REG(DP_TP_CTL(con->ddi_id)) = dp_tp_ctl_flags | (REG(DP_TP_CTL(con->ddi_id)) & 0xFFFBF8FF);
	REG(DP_TP_CTL(con->ddi_id)) |= DP_TP_CTL_ENABLE;
	REG(DDI_BUF_CTL(con->ddi_id)) = DDI_BUF_CTL_DP_PORT_WIDTH(max_lanes) | (REG(DDI_BUF_CTL(con->ddi_id)) & ~DDI_BUF_CTL_DP_PORT_WIDTH_MASK);

	if(con->type == EDP) {
		uint32_t dispio_cr_tx_bmu_cr0 = 0;

		if(enc->edp.edp_port_reversal) {
			REG(DDI_BUF_CTL(con->ddi_id)) |= DDI_BUF_CTL_PORT_REVERSAL;
		}

		uint8_t balance_leg = 0;

		if(enc->edp.edp_iboost) {
			balance_leg = enc->edp.edp_balance_leg_val;
		} else if(enc->edp.edp_vswing_preemph == 1) {
			if(gpu->variant == ULX && gpu->subgen == SUBGEN_KABY_LAKE) {
				balance_leg = 3;
			} else {
				balance_leg = 1;
			}
		}

		kbl::ddi::balance_leg_set(gpu, con->ddi_id, balance_leg);

		if(max_lanes == 4)
			REG(DISPIO_CR_TX_BMU_CR0) = (REG(DISPIO_CR_TX_BMU_CR0) & ~DISPIO_CR_TX_BMU_CR0_DDI_BALANCE_LEG_MASK(DDI_E)) | DISPIO_CR_TX_BMU_CR0_DDI_BALANCE_LEG(DDI_E, balance_leg);
	}

	REG(DDI_BUF_CTL(con->ddi_id)) |= DDI_BUF_CTL_ENABLE;
	lil_usleep(518);

	if(con->type == EDP) {
		if(enc->edp.edp_fast_link_training_supported) {
			lil_panic("fast eDP link training unsupported");
		} else {
			if(!kbl::link_training::edp(gpu, crtc, enc->edp.edp_max_link_rate, enc->edp.edp_lane_count))
				lil_panic("eDP link training failed");
		}
	} else {
		if(!kbl::link_training::edp(gpu, crtc, enc->dp.dp_max_link_rate, enc->dp.dp_lane_count))
			lil_panic("DP link training failed");
	}

	uint32_t htotal = crtc->current_mode.htotal;
	uint32_t pixel_clock = crtc->current_mode.clock;
	uint32_t pitch = (crtc->current_mode.hactive * 4 + 63) & 0xFFFFFFC0;
	uint32_t linetime = div_u32_ceil(8000 * htotal, pixel_clock);

	REG(DP_TP_CTL(con->ddi_id)) = (REG(DP_TP_CTL(con->ddi_id)) & 0xFFFFF8FF) | 0x300;
	kbl::transcoder::configure_clock(gpu, crtc);
	REG(WM_LINETIME(crtc->pipe_id)) = (REG(WM_LINETIME(crtc->pipe_id)) & 0xFFFFFE00) | (linetime & 0x1ff);
	kbl::pipe::src_size_set(gpu, crtc);
	kbl::plane::size_set(gpu, crtc);

	enable_vblank(gpu, crtc);
	kbl::transcoder::timings_configure(gpu, crtc);
	kbl::transcoder::bpp_set(gpu, crtc, bpp);
	kbl::pipe::dithering_enable(gpu, crtc, bpp);
	kbl::transcoder::configure_m_n(gpu, crtc, crtc->current_mode.clock, link_rate, max_lanes, bpp);
	kbl::transcoder::ddi_polarity_setup(gpu, crtc);
	kbl::transcoder::set_dp_msa_misc(gpu, crtc, bpp);
	kbl::transcoder::ddi_setup(gpu, crtc, max_lanes);
	kbl::transcoder::enable(gpu, crtc);
	if(crtc->planes[0].enabled)
		kbl::plane::enable(gpu, crtc, true);

	if(con->type == EDP) {
		if(enc->edp.t8 > enc->edp.pwm_on_to_backlight_enable) {
			lil_usleep(100 * (enc->edp.t8 - enc->edp.pwm_on_to_backlight_enable));
		}

		if(enc->edp.backlight_control_method_type == 2) {
			uint32_t blc_pwm_pipe = 0;

			if(crtc->pipe_id == 1) {
				blc_pwm_pipe = 0x20000000;
			} else if(crtc->pipe_id == 2) {
				blc_pwm_pipe = 0x40000000;
			}

			REG(BLC_PWM_CTL) = (REG(BLC_PWM_CTL) & 0x9FFFFFFF) | blc_pwm_pipe;
			REG(BLC_PWM_CTL) |= BLC_PWM_CTL_PWM_ENABLE;
			REG(SBLC_PWM_CTL1) |= SBLC_PWM_CTL1_PWM_PCH_ENABLE;

			lil_usleep(1000000 / enc->edp.pwm_inv_freq + 100 * enc->edp.pwm_on_to_backlight_enable);

			REG(PP_CONTROL) |= 4;
		}
	}
}

void crtc_shutdown(LilGpu *gpu, LilCrtc *crtc) {
	switch(crtc->connector->type) {
		case DISPLAYPORT:
		case EDP: {
			kbl::plane::disable(gpu, crtc);
			kbl::transcoder::disable(gpu, crtc->transcoder);
			kbl::transcoder::ddi_disable(gpu, crtc->transcoder);
			kbl::pipe::scaler_disable(gpu, crtc);
			kbl::transcoder::clock_disable(gpu, crtc);

			REG(DDI_BUF_CTL(crtc->connector->ddi_id)) &= ~DDI_BUF_CTL_PORT_REVERSAL;
			REG(DDI_BUF_CTL(crtc->connector->ddi_id)) &= ~DDI_BUF_CTL_ENABLE;

			kbl::ddi::balance_leg_set(gpu, crtc->connector->ddi_id, 0);

			REG(DP_TP_CTL(crtc->connector->ddi_id)) &= ~0x10000;
			lil_usleep(10);
			// kbl::ddi::power_disable(gpu, crtc->connector);
			kbl::ddi::clock_disable(gpu, crtc);

			if(crtc->connector->type == DISPLAYPORT) {
				kbl::pll::disable(gpu, crtc->connector);
			}
			break;
		}
		default: {
			lil_panic("connector type unhandled");
		}
	}
}

}
