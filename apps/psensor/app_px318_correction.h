
#ifndef __PX318_correction_H__
#define __PX318_correction_H__


//Fixed-point
#ifndef FIXEDPT_BITS
#define FIXEDPT_BITS 32
#endif
#include <stdint.h>



#ifdef __cplusplus
extern "C" {
#endif
#include "hal_key.h"


#ifdef __FJH_PX318_CUR_DELAY__
void PX318J_correction_threadhandle_open(void);
#endif


uint32_t PX318J_correction_open(void);
uint32_t PX318J_correction_close(void);
void PX318J_correction_earin(void);
void PX318J_correction_earout(void);
void Px318j_correction_cor_init(uint8_t PsCtGain_t,uint8_t PsCtDac_t,uint16_t PsCal_t,bool PsCorrectionFlg_t,uint16_t PsThreHigh_t,bool PsThreHighFlg_t,uint16_t PsThreLow_t,bool PsThreLowFlg_t);



#ifdef __cplusplus
}
#endif



//Function form MCU
uint32_t MCU_I2C_Write(uint8_t devid, uint8_t reg, uint8_t* data, uint8_t num);
uint32_t MCU_I2C_Read(uint8_t devid, uint8_t reg, uint8_t* data, uint8_t num);
//extern void MCU_Delay_ms(uint32_t millisecond);
#define PsBits (0x00) //9-bits
#define PsMean (0x00) //Mean 1
#define PsCtrl ((PsBits << 4) | (PsMean << 6) | (0x05))
#define PsPuw (0x0a) // 32 width = 64 us
#define PsPuc (0x02) // 2 count
#define PsDrv (0x0B) //12 mA
#define PsDrvCtrl (PsDrv)
#define WaitTime (0x11) //170 ms
#define PsWaitAlgo (0x01)
#define PsIntAlgo (0x01)
#define PsPers (0x04)
//PsInt asserted after 4 consecutive PsData meets the PsInt criteria
#define PsAlgoCtrl ((PsWaitAlgo << 5) | (PsIntAlgo << 4) | (PsPers))
//#define PsThreHigh 400
//#define PsThreLow 200
#define PXY_FULL_RANGE ((1 << (PsBits + 9)) - 1)
#define TARGET_PXY ((PXY_FULL_RANGE + 1) >> 2)
void px318j_enable(uint8_t addr, uint8_t enable);
uint8_t px318j_auto_dac(uint8_t addr);





#if FIXEDPT_BITS == 16
typedef int16_t fixedpt;
typedef int32_t fixedptd;
typedef uint16_t fixedptu;
typedef uint32_t fixedptud;
#elif FIXEDPT_BITS == 32
typedef int32_t fixedpt;
typedef int64_t fixedptd;
typedef uint32_t fixedptu;
typedef uint64_t fixedptud;
#elif FIXEDPT_BITS == 64
typedef int64_t fixedpt;
typedef __int128_t fixedptd;
typedef uint64_t fixedptu;
typedef __uint128_t fixedptud;
#else
#error "FIXEDPT_BITS must be equal to 16, 32 or 64"
#endif
#ifndef FIXEDPT_WBITS
#define FIXEDPT_WBITS 16
#endif
#if FIXEDPT_WBITS >= FIXEDPT_BITS
#error "FIXEDPT_WBITS must be less than or equal to FIXEDPT_BITS"
#endif
#define FIXEDPT_FBITS (FIXEDPT_BITS - FIXEDPT_WBITS)
#define FIXEDPT_FMASK (((fixedpt)1 << FIXEDPT_FBITS) - 1)
#define fixedpt_rconst(R) ((fixedpt)((R) * FIXEDPT_ONE + ((R) >= 0 ? 0.5 : -0.5)))
#define fixedpt_fromint(I) ((fixedptd)(I) << FIXEDPT_FBITS)
#define fixedpt_toint(F) ((F) >> FIXEDPT_FBITS)
#define fixedpt_add(A,B) ((A) + (B))
#define fixedpt_sub(A,B) ((A) - (B))
#define fixedpt_xmul(A,B) ((fixedpt)(((fixedptd)(A) * (fixedptd)(B)) >> FIXEDPT_FBITS))
#define fixedpt_xdiv(A,B) ((fixedpt)(((fixedptd)(A) << FIXEDPT_FBITS) / (fixedptd)(B)))
#define fixedpt_fracpart(A) ((fixedpt)(A) & FIXEDPT_FMASK)
#define FIXEDPT_ONE ((fixedpt)((fixedpt)1 << FIXEDPT_FBITS))
#define FIXEDPT_ONE_HALF (FIXEDPT_ONE >> 1)





#endif

