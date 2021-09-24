#ifndef __SOUNDPLUS_ADAPTER_H__
#define __SOUNDPLUS_ADAPTER_H__
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif
/***************************************************************
    FunctionName:   sndp_auth_get_bt_address
    Purpose:        support soundplus algorithm authorize
                    call this function in factory_section.c
    Parameter:
                    version: the version of nvrec

    Return:         void
****************************************************************/
void sndp_auth_get_bt_address(uint8_t version);
/***************************************************************
    FunctionName:   sndp_auth_get_norflash_uuid
    Purpose:        support soundplus algorithm authorize
                    call this function in norflash_drv.c
                    like other function norflash_read_reg_ex
    Parameter:
                    cmd:
                    param:
                    param_len:
                    val:
                    len:
    Return:         return 0 if read sucessed
****************************************************************/
int sndp_auth_get_norflash_uuid(uint8_t cmd, uint8_t* param, uint32_t param_len, uint8_t* val, uint32_t len);
/***************************************************************
    FunctionName:   soundplus_auth
    Purpose:        soundplus algorithm authorize
    Parameter:
                    Key:
                    Key_len:
    Return:         return 1 if auth sucessed,else return error code
****************************************************************/
int soundplus_auth(uint8_t* key, int Key_len);
/***************************************************************
    FunctionName:   soundplus_auth_status
    Purpose:        get soundplus algorithm license state
    Parameter:
                    
    Return:         return 1 if auth sucessed,else return error code
****************************************************************/
int soundplus_auth_status();

/***************************************************************
    FunctionName:   soundplus_deal_Tx
    Purpose:        soundplus algorithm Tx ENC/AEC process
    Parameter:      
                    buf : mic pcm data.
                    ref : ref pcm data.
                    buf_len : mic pcm numsamples,
                          set 240*mic_num when BTIF_HF_SCO_CODEC_MSBC,
                          set 120*mic_num when BTIF_HF_SCO_CODEC_CVSD
                    ref_len : ref pcm numsamples,
                          set 240 when BTIF_HF_SCO_CODEC_MSBC,
                          set 120 when BTIF_HF_SCO_CODEC_CVSD
    Return:         return 0 if sucess,else return other
****************************************************************/
int soundplus_deal_Tx(short *buf, short *ref, int buf_len, int ref_len);

/***************************************************************
    FunctionName:   soundplus_deal_Rx
    Purpose:        soundplus algorithm Rx process
    Parameter:      
                    buf : bt Rx pcm data.
                    len : Rx pcm data numsamples,
                          set 240 when BTIF_HF_SCO_CODEC_MSBC,
                          set 120 when BTIF_HF_SCO_CODEC_CVSD
    Return:         return 0 if sucess,else return other
****************************************************************/
int soundplus_deal_Rx(short *inX, int len);

/***************************************************************
    FunctionName:   soundplus_init
    Purpose:        soundplus algorithm init
    Parameter:      
                    NrwFlag : narrowband flag. 
                              set 1 when BTIF_HF_SCO_CODEC_CVSD, 
                              set 0 when BTIF_HF_SCO_CODEC_MSBC
    Return:         return 0 if sucess,else return other
****************************************************************/
int soundplus_init(int NrwFlag);

/***************************************************************
    FunctionName:   soundplus_deinit
    Purpose:        soundplus algorithm deinit
    Parameter:      
                    void
    Return:         return 0 if sucess,else return other
****************************************************************/
int soundplus_deinit(void);

#ifdef __cplusplus
}
#endif

#endif
