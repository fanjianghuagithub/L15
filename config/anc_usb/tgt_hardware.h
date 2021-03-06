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
#ifndef __TGT_HARDWARE__
#define __TGT_HARDWARE__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_iomux.h"
#include "hal_gpio.h"
#include "hal_key.h"
#include "hal_aud.h"

//config hwardware codec iir.
#define EQ_HW_DAC_IIR_LIST_NUM              1
#define EQ_HW_ADC_IIR_LIST_NUM              1
#define EQ_HW_IIR_LIST_NUM                  2
#define EQ_SW_IIR_LIST_NUM                  2
#define EQ_HW_FIR_LIST_NUM                  8

//pwl
#define CFG_HW_PLW_NUM (0)
extern const struct HAL_IOMUX_PIN_FUNCTION_MAP cfg_hw_pinmux_pwl[CFG_HW_PLW_NUM];

//adckey define
#define CFG_HW_ADCKEY_NUMBER 9
#define CFG_HW_ADCKEY_BASE 0
#define CFG_HW_ADCKEY_ADC_MAXVOLT 1000
#define CFG_HW_ADCKEY_ADC_MINVOLT 0
#define CFG_HW_ADCKEY_ADC_KEYVOLT_BASE 130
extern const uint16_t CFG_HW_ADCKEY_MAP_TABLE[CFG_HW_ADCKEY_NUMBER];

#define BTA_AV_CO_SBC_MAX_BITPOOL  53

//gpiokey define
#ifdef CHIP_BEST2000
#define CFG_HW_GPIOKEY_NUM (5)
#else
#define CFG_HW_GPIOKEY_NUM (0)
#endif
extern const struct HAL_KEY_GPIOKEY_CFG_T cfg_hw_gpio_key_cfg[CFG_HW_GPIOKEY_NUM];

//#define PA_ON_OFF_KEY                       HAL_KEY_CODE_FN7
//#define PERF_TEST_POWER_KEY                 HAL_KEY_CODE_FN8

// ANC function key
#define ANC_FUNCTION_KEY                    HAL_KEY_CODE_PWR

// #define ANC_SWITCH_GPIO_PIN                 HAL_IOMUX_PIN_P1_3

// #define ANC_SWITCH_GPADC_CHAN               HAL_GPADC_CHAN_3
// #define ANC_SWITCH_VOLTAGE_THRESHOLD        1600
// #define ANC_SWITCH_LEVEL_HIGH               0
// #define ANC_SWITCH_LEVEL_LOW                1

// ANC startup timeout
//#define ANC_INIT_ON_TIMEOUT_MS              1200

// ANC coefficient curve number
#define ANC_COEF_NUM                        (1)

//#define ANC_TALK_THROUGH

#ifdef ANC_TALK_THROUGH
#define ANC_COEF_LIST_NUM                   (ANC_COEF_NUM + 1)
#else
#define ANC_COEF_LIST_NUM                   (ANC_COEF_NUM)
#endif

#ifdef ANC_PROD_TEST
extern enum AUD_CHANNEL_MAP_T anc_ff_mic_ch_l;
extern enum AUD_CHANNEL_MAP_T anc_ff_mic_ch_r;
extern enum AUD_CHANNEL_MAP_T anc_fb_mic_ch_l;
extern enum AUD_CHANNEL_MAP_T anc_fb_mic_ch_r;

#define ANC_FF_MIC_CH_L                     anc_ff_mic_ch_l
#define ANC_FF_MIC_CH_R                     anc_ff_mic_ch_r
#define ANC_FB_MIC_CH_L                     anc_fb_mic_ch_l
#define ANC_FB_MIC_CH_R                     anc_fb_mic_ch_r

#define ANALOG_ADC_A_GAIN_DB                DEFAULT_ANC_FF_ADC_GAIN_DB
#define ANALOG_ADC_B_GAIN_DB                DEFAULT_ANC_FF_ADC_GAIN_DB
#define ANALOG_ADC_C_GAIN_DB                DEFAULT_ANC_FF_ADC_GAIN_DB
#define ANALOG_ADC_D_GAIN_DB                DEFAULT_ANC_FF_ADC_GAIN_DB
#define ANALOG_ADC_E_GAIN_DB                DEFAULT_ANC_FF_ADC_GAIN_DB

#else
#define ANC_FF_MIC_CH_L                     AUD_CHANNEL_MAP_CH0
#define ANC_FF_MIC_CH_R                     AUD_CHANNEL_MAP_CH1
#define ANC_FB_MIC_CH_L                     AUD_CHANNEL_MAP_CH2
#define ANC_FB_MIC_CH_R                     AUD_CHANNEL_MAP_CH3
#endif

#ifdef ANC_PROD_TEST
#define ANC_VMIC_CFG                        (AUD_VMIC_MAP_VMIC1|AUD_VMIC_MAP_VMIC2|AUD_VMIC_MAP_VMIC3|AUD_VMIC_MAP_VMIC3|AUD_VMIC_MAP_VMIC4)
#else
#define ANC_VMIC_CFG                        (AUD_VMIC_MAP_VMIC1)
#endif

// audio codec
#define CFG_HW_AUD_INPUT_PATH_NUM           2
extern const struct AUD_IO_PATH_CFG_T cfg_audio_input_path_cfg[CFG_HW_AUD_INPUT_PATH_NUM];

#define CFG_HW_AUD_OUTPUT_PATH_SPEAKER_DEV  (AUD_CHANNEL_MAP_CH0|AUD_CHANNEL_MAP_CH1)

#define CODEC_OUTPUT_DEV                    CFG_HW_AUD_OUTPUT_PATH_SPEAKER_DEV

#define CFG_HW_AUD_SIDETONE_MIC_DEV         (AUD_CHANNEL_MAP_CH0)
#define CFG_HW_AUD_SIDETONE_GAIN_DBVAL      (-20)
//#define CFG_HW_AUD_SIDETONE_IIR_INDEX       (1)

//#define CFG_HW_AUD_OUTPUT_POP_SWITCH        HAL_GPIO_PIN_P1_3

//bt config
extern const char *BT_LOCAL_NAME;
extern const char *BLE_DEFAULT_NAME;
extern uint8_t ble_addr[6];
extern uint8_t bt_addr[6];

#ifdef ANC_PROD_TEST
#define CODEC_SADC_VOL (1)
#else
#define CODEC_SADC_VOL (7)
#endif
extern const struct CODEC_DAC_VOL_T codec_dac_vol[TGT_VOLUME_LEVEL_QTY];

//range -12~+12
#define CFG_HW_AUD_EQ_NUM_BANDS (8)
extern const int8_t cfg_hw_aud_eq_band_settings[CFG_HW_AUD_EQ_NUM_BANDS];

//battery info
#define APP_BATTERY_MIN_MV (3200)
#define APP_BATTERY_PD_MV   (3100)

#define APP_BATTERY_MAX_MV (4150)

extern const struct HAL_IOMUX_PIN_FUNCTION_MAP app_battery_ext_charger_enable_cfg;
extern const struct HAL_IOMUX_PIN_FUNCTION_MAP app_battery_ext_charger_detecter_cfg;

#ifdef CFG_MIC_KEY

#define MIC_KEY_NUM     3

typedef struct {
    uint16_t    ref_vol_low;    // in mv
    uint16_t    ref_vol_high;   // in mv
    uint16_t    hid_evt;
} MIC_KEY_CFG_T;

extern const MIC_KEY_CFG_T mic_key_cfg_lst[MIC_KEY_NUM];
extern const enum HAL_GPIO_PIN_T mic_key_det_gpio_pin;
extern const enum HAL_GPADC_CHAN_T mic_key_gpadc_chan;

#endif // CFG_MIC_KEY

#ifdef __cplusplus
}
#endif

#endif
