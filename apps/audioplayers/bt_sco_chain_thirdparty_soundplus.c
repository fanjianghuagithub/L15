#include "bt_sco_chain.h"
#include "speech_memory.h"
#include "speech_utils.h"
#include "hal_trace.h"
#include "audio_dump.h"
#include "soundplus_adapter.h"
#include "hal_timer.h"


#define FRAME_LEN_16K 240
#define FRAME_LEN_8K 120
short *aec_echo_buf = NULL;

#if defined(MSBC_8K_SAMPLE_RATE)
#define SPEECH_HEAP_RESERVE_SIZE (1024 * 60)
#else
#define SPEECH_HEAP_RESERVE_SIZE (1024 * 60)
#endif

// Use to free buffer
static short *aec_echo_buf_ptr;

int speech_init(int tx_sample_rate, int rx_sample_rate,
                     int tx_frame_ms, int rx_frame_ms,
                     int sco_frame_ms,
                     uint8_t *buf, int len)
{
    //speech_heap_init(buf, SPEECH_HEAP_RESERVE_SIZE);
    speech_heap_init(buf, len);
    TRACE(1,"speech_init111111111111111111111111111111");
    int frame_len = SPEECH_FRAME_MS_TO_LEN(tx_sample_rate, tx_frame_ms);

    aec_echo_buf = (short *)speech_calloc(frame_len, sizeof(short));
    aec_echo_buf_ptr = aec_echo_buf;

    if(frame_len == FRAME_LEN_16K)
    {
        soundplus_init(0);
    }
    else if(frame_len == FRAME_LEN_8K)
    {
        soundplus_init(1);
    }
    else
    {
        //TRACE("ERROR! frame_len=%d", frame_len);
        return -1;
    }

	//int system_freq = hal_sys_timer_calc_cpu_freq(5, 0);
	//TRACE("rex sys freq calc : %d\n", system_freq);
	
#if defined(SNDP_TX_DUMP_ENABLE)
    audio_dump_init(frame_len, sizeof(int16_t), 4);
#endif

    return 0;
}

int speech_deinit(void)
{
    speech_free(aec_echo_buf_ptr);
    soundplus_deinit();

    size_t total = 0, used = 0, max_used = 0;
    speech_memory_info(&total, &used, &max_used);
    TRACE(3,"SPEECH MALLOC MEM: total - %d, used - %d, max_used - %d.", total, used, max_used);
    ASSERT(used == 0, "[%s] used != 0", __func__);

    return 0;
}

int speech_tx_process(void *pcm_buf, void *ref_buf, int *pcm_len)
{
    int16_t *pcm16_buf = (int16_t *)pcm_buf;
    //int16_t *ref16_buf = (int16_t *)ref_buf;
    int pcm16_len = *pcm_len;
    TRACE(1,"pcm16_len=%d", pcm16_len);
#if defined(SNDP_TX_DUMP_ENABLE)
    short tmp[240] = {0};
    audio_dump_clear_up();
    for(int i=0; i<pcm16_len/2; i++)
        tmp[i] = pcm16_buf[i*2+1];
    audio_dump_add_channel_data(0, tmp, pcm16_len / SPEECH_CODEC_CAPTURE_CHANNEL_NUM);
    for(int i=0; i<pcm16_len/2; i++)
        tmp[i] = pcm16_buf[i*2];
    audio_dump_add_channel_data(1, tmp, pcm16_len / SPEECH_CODEC_CAPTURE_CHANNEL_NUM);
    audio_dump_add_channel_data(2, aec_echo_buf_ptr, pcm16_len / SPEECH_CODEC_CAPTURE_CHANNEL_NUM);
    uint32_t start_ticks = hal_fast_sys_timer_get();
    soundplus_deal_Tx(pcm16_buf, aec_echo_buf_ptr, pcm16_len, pcm16_len/SPEECH_CODEC_CAPTURE_CHANNEL_NUM);
    uint32_t end_ticks = hal_fast_sys_timer_get();
    TRACE(1,"soundplus_deal_Tx] takes = [%d] us", FAST_TICKS_TO_US(end_ticks - start_ticks));
    audio_dump_add_channel_data(3, pcm16_buf, pcm16_len / SPEECH_CODEC_CAPTURE_CHANNEL_NUM);
    audio_dump_run();
#else
    uint32_t start_ticks = hal_fast_sys_timer_get();
    soundplus_deal_Tx(pcm16_buf, aec_echo_buf_ptr, pcm16_len, pcm16_len/2);
    uint32_t end_ticks = hal_fast_sys_timer_get();
    TRACE(3,"soundplus_deal_Tx] takes = [%d] us", FAST_TICKS_TO_US(end_ticks - start_ticks));

#endif
    
    *pcm_len = pcm16_len/2;

    return 0;
}

int speech_rx_process(void *pcm_buf, int *pcm_len)
{
    TRACE(1,"speech_rx_process pcm_len=%d", *pcm_len);
    memcpy(aec_echo_buf_ptr, pcm_buf, (*pcm_len)*2);
    return 0;
}
