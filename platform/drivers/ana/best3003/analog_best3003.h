/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
#ifndef __ANALOG_BEST3001_H__
#define __ANALOG_BEST3001_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "dco_best3003.h"
#ifdef DONGLE_SUPPORT
#include "dongle_best3003.h"
#endif

#define ISPI_ANA_REG(reg)                   (((reg) & 0xFFF) | 0x1000)      // cs = 1, analog interface

#define MAX_ANA_MIC_CH_NUM  5

enum ANA_CODEC_USER_T {
    ANA_CODEC_USER_DAC      = (1 << 0),
    ANA_CODEC_USER_ADC      = (1 << 1),

    ANA_CODEC_USER_CODEC    = (1 << 2),
    ANA_CODEC_USER_MICKEY   = (1 << 3),

    ANA_CODEC_USER_ANC_FF   = (1 << 4),
    ANA_CODEC_USER_ANC_FB   = (1 << 5),

    ANA_CODEC_USER_DONGLE   = (1 << 6),
};

void analog_osc_clk_default_enable (void);
void analog_codec_clk_default_sel (void);
void analog_aud_set_sar_adc_sample_rate (enum AUD_SAMPRATE_T sample_rate);

uint32_t analog_aud_get_max_dre_gain(void);

void analog_aud_enable_vmic(enum ANA_CODEC_USER_T user, uint32_t dev);

#ifdef __cplusplus
}
#endif

#endif

