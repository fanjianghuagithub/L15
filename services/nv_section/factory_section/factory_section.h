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
#ifndef __FACTORY_SECTIONS_H__
#define __FACTORY_SECTIONS_H__

#define ALIGN4 __attribute__((aligned(4)))
#define nvrec_mini_version      1
#define nvrec_dev_magic         0xba80
#define nvrec_current_version   2
#define FACTORY_SECTOR_SIZE     4096
typedef struct{
    unsigned short magic;
    unsigned short version;
    unsigned int crc ;
    unsigned int reserved0;
    unsigned int reserved1;
}section_head_t;



#ifdef __FJH_P_SENSOR_CORRECTION__
struct p_sensor_t2
{
    uint8_t PsCtGain;
	uint8_t PsCtDac;
    uint16_t PsCal;
	bool PsCorrectionflg;
	uint16_t PsThreHigh_P;
	bool PsThreHighflg;
	uint16_t PsThreLow_p;
	bool PsThreLowflg;	
};
#endif


typedef struct{
    unsigned char device_name[248+1] ALIGN4;
    unsigned char bt_address[8] ALIGN4;
    unsigned char ble_address[8] ALIGN4;
    unsigned char tester_address[8] ALIGN4;
    unsigned int  xtal_fcap ALIGN4;
    unsigned int  rev1_data_len;

    unsigned int  rev2_data_len;
    unsigned int  rev2_crc;
	unsigned int  rev2_reserved0;
	unsigned int  rev2_reserved1;
	unsigned int  rev2_bt_name[63];
	unsigned int  rev2_bt_addr[2];
	unsigned int  rev2_ble_addr[2];
	unsigned int  rev2_dongle_addr[2];
	unsigned int  rev2_xtal_fcap;
	unsigned int  rev2_ble_name[8];
	struct p_sensor_t2 p_sensor2;

}factory_section_data_t;

typedef struct{
    section_head_t head;
    factory_section_data_t data;
}factory_section_t;

#ifdef __cplusplus
extern "C" {
#endif

void factory_section_init(void);
int factory_section_open(void);
void factory_section_original_btaddr_get(uint8_t *btAddr);
int factory_section_xtal_fcap_get(unsigned int *xtal_fcap);
int factory_section_xtal_fcap_set(unsigned int xtal_fcap);
uint8_t* factory_section_get_bt_address(void);
uint8_t* factory_section_get_bt_name(void);
uint8_t* factory_section_get_ble_name(void);
uint32_t factory_section_get_version(void);
bool factory_get_px318data_flg(void);
int factory_px318_data_set(uint8_t PsCtGain_t,uint8_t PsCtDac_t,uint16_t PsCal_t,bool PsCorrectionFlg_t,uint16_t PsThreHigh_t,bool PsThreHighFlg_t,uint16_t PsThreLow_t,bool PsThreLowFlg_t);
uint8_t factory_get_px318data_PsCtGain_t(void);
uint8_t factory_get_px318data_PsCtDac_t(void);
uint8_t factory_get_px318data_PsCal_t(void);
uint16_t factory_get_px318data_PsThreHigh_P(void);
bool factory_get_px318data_PsThreHighflg(void);
uint16_t factory_get_px318data_PsThreLow_p(void);
bool factory_get_px318data_PsThreLowflg(void);

#ifdef __cplusplus
}
#endif
#endif
