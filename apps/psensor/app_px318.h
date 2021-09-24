
#ifndef __PX318_H__
#define __PX318_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "hal_key.h"
uint32_t PX318J_open(void);
void PX318J_get_val();
uint32_t PX318J_close(void);
void PX318J_get_val();

#ifdef __FJH_P_SENSOR_CORRECTION__
void Px318j_correction_init(uint8_t PsCtGain_t,uint8_t PsCtDac_t,uint16_t PsCal_t,bool PsCorrectionFlg_t,uint16_t PsThreHigh_t,bool PsThreHighFlg_t,uint16_t PsThreLow_t,bool PsThreLowFlg_t);
//void nv_test(void);
int I2C_WriteByte(uint8_t reg_addr, uint8_t value);
int I2C_WriteWord(uint8_t reg_addr, uint16_t value);
int I2C_ReadByte(uint8_t reg_addr, uint8_t *value);
int I2C_ReadWord(uint8_t reg_addr, uint16_t *value);



//extern struct P_SENSOR_DATA P_sensor_data;
#endif

#ifdef __cplusplus
}
#endif

#endif

