#ifndef _SNDP_LIB_H_
#define _SNDP_LIB_H_

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

//-----------------------------------------------------------------------------

/***************************************************************
	FunctionName:	sndp_license_auth
	Purpose:		soundplus algorithm authorization
	Parameter:		[IN] license_key: license byte array
					[IN] Key_len: byte array length
					
	Return:			return 1 if sucess,else return error code
****************************************************************/
int sndp_license_auth(uint8_t* license_key, int Key_len);

/***************************************************************
    FunctionName:   sndp_license_status_get
    Purpose:        get soundplus algorithm license status
    Parameter:
                    [IN] void
    Return:         return 1 if Auth sucess,else return error code
****************************************************************/
int sndp_license_status_get(void);

/******************************************************************************
    FunctionName:   Sndp_get_alg_ver
    Purpose:        get soundplus algorithm version
    Parameter:      
                    void
            
    Return:         char *:  point to version-string
******************************************************************************/
char *Sndp_get_alg_ver(void);

/******************************************************************************
    FunctionName:   Sndp_SpxEnh_MemSize
    Purpose:        calculate soundplus algorithm required memory size
    Parameter:      
                    NrwFlag  : narrowband flag. 
							   set 1 when BTIF_HF_SCO_CODEC_CVSD 
							   set 0 when BTIF_HF_SCO_CODEC_MSBC
					int numchannels_mic	: numbers of micChn	
    Return:         int: memory size in bytes
******************************************************************************/
int Sndp_SpxEnh_MemSize(int NrwFlag);

/******************************************************************************
    FunctionName:   Sndp_SpxEnh_Init
    Purpose:        soundplus algorithm init
    Parameter:      
					void *membuf : the memory giving to alg
					float PostGmin : speech enhancement gain    //20200814 @cwk
    Return:         void
******************************************************************************/
void Sndp_SpxEnh_Init(void *membuf, float PostGmin);

/******************************************************************************
	FunctionName:	Sndp_SpxEnh_Tx
	Purpose:		soundplus algorithm process main-function
	Parameter:		
					float *outY : one-channel output data
					float *inX 	: two-channel de-interleaved input data
					float *ref  : ref data
					int numsamples_mic : frame len of mic
									 set 120 when BTIF_HF_SCO_CODEC_CVSD 
                                     set 240 when BTIF_HF_SCO_CODEC_MSBC
					int numsamples_ref : frame len of ref
					                 set 120 when BTIF_HF_SCO_CODEC_CVSD 
                                     set 240 when BTIF_HF_SCO_CODEC_MSBC
	Return: 		void
******************************************************************************/
void Sndp_SpxEnh_Tx(float *outY, float *inX, float *ref, int numsamples_mic, int numsamples_ref);

/******************************************************************************
	FunctionName:	Sndp_SpxEnh_Rx
	Purpose:		soundplus algorithm process main-function for Rx
	Parameter:		
					float *outY : one-channel output data
					float *inX	: one-channel input  data
					int numsamples : frame len
					                 set 120 when BTIF_HF_SCO_CODEC_CVSD 
                                     set 240 when BTIF_HF_SCO_CODEC_MSBC
	Return: 		void
******************************************************************************/
void Sndp_SpxEnh_Rx(float *outY, float *inX, int numsamples);

#ifdef __cplusplus
}
#endif

#endif // _SNDP_LIB_H
