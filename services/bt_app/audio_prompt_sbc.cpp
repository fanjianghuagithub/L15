#include "audio_prompt_sbc.h"
#include "hal_trace.h"
#include "string.h"
#include "app_bt_stream.h"
#include "app_bt_media_manager.h"
#include "app_utils.h"
#include "cmsis.h"
#include "avdtp_i.h"

#include "bt_drv_interface.h"
#include "audioflinger.h"
#include "app_media_player.h"
#if defined(IBRT)
#include "app_tws_ibrt.h"
#include "app_tws_ibrt_cmd_handler.h"
#include "app_tws_ctrl_thread.h"
#include "app_ibrt_if.h"
#endif
#include "app_audio.h"
#include "apps.h"
#include "app_bt.h"
#ifdef MIX_AUDIO_PROMPT_WITH_A2DP_MEDIA_ENABLED

#define AUDIO_PROMPT_RESAMPLE_ITER_NUM               256

#ifdef TWS_PROMPT_SYNC
extern void tws_sync_mix_prompt_start_handling(void);
extern void tws_reset_mix_prompt_trigger_ticks(void);
extern void tws_enable_mix_prompt(bool isEnable);
#endif


extern void app_stop_a2dp_media_stream(uint8_t devId);
extern void app_stop_sco_media_stream(uint8_t devId);
extern void app_bt_stream_copy_track_one_to_two_16bits(int16_t *dst_buf, int16_t *src_buf, uint32_t src_len);

static uint32_t audio_prompt_sbc_decode(uint8_t* pcm_buffer, uint32_t expectedOutputSize, uint8_t isReset);

static btif_sbc_decoder_t audio_prompt_sbc_decoder;

static float audio_prompt_sbc_eq_band_gain[8] = {1, 1, 1, 1, 1, 1, 1, 1};

// keep the current volume
#define KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT          -1

// from TGT_VOLUME_LEVEL_T or KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT
#define DEFAULT_VOLUME_FOR_MIX_PROMPT               KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT

#define DEFAULT_COEFF_FOR_MIX_PROMPT_FOR_MUSIC      1.0
#define DEFAULT_COEFF_FOR_MIX_MUSIC_FOR_MUSIC       0.4
#define DEFAULT_COEFF_FOR_MIX_PROMPT_FOR_CALL       1.0
#define DEFAULT_COEFF_FOR_MIX_CALL_FOR_CALL         0.4

static int audio_prompt_sbc_init_decoder(void)
{
    btif_sbc_init_decoder(&audio_prompt_sbc_decoder);
    return 0;
}

typedef struct
{
    uint32_t    wholeEncodedDataLen;
    uint32_t    leftEncodedDataLen;
    uint8_t*    promptDataBuf;
    uint16_t    tmpSourcePcmDataLen;
    uint16_t    tmpSourcePcmDataOutIndex;
    uint8_t*    tmpSourcePcmDataBuf;
    uint8_t*    tmpTargetPcmDataBuf;
    CQueue      pcmDataQueue;
    uint8_t     isResetDecoder;
    uint8_t     isAudioPromptDecodingDone;
    uint8_t     targetBytesCntPerSample;
    uint32_t    targetSampleRate;
    struct APP_RESAMPLE_T * resampler;
    uint32_t    targetPcmChunkSize;
    float       resampleRatio;
    uint8_t*    bufForResampler;
    uint32_t    resampleBufLen;
    uint8_t     pendingStopOp;
    int8_t      savedStoppedStreamId;
    uint8_t     targetChannelCnt;
    uint8_t     mixType;
    uint16_t    promptId;
    int16_t     volume_level_override;
    float       coeff_for_mix_prompt_for_music;
    float       coeff_for_mix_music_for_music;
    float       coeff_for_mix_prompt_for_call;
    float       coeff_for_mix_call_for_call;
    uint8_t     isMixPromptOn;
    int16_t     currentVol;
} AUDIO_PROMPT_ENV_T;

static AUDIO_PROMPT_ENV_T audio_prompt_env;

static void audio_prompt_set_pending_stop_op(uint8_t op)
{
    audio_prompt_env.pendingStopOp = op;
    TRACE(1,"pendingStopOp is set to %d", op);
}

static uint8_t audio_prompt_get_pending_stop_op(void)
{
    return audio_prompt_env.pendingStopOp;
}

static void audio_prompt_set_saved_stopped_stream_id(int8_t id)
{
    audio_prompt_env.savedStoppedStreamId = id;
    TRACE(1,"savedStoppedStreamId is set to %d", id);
}

static int8_t audio_prompt_get_saved_stopped_stream_id(void)
{
    return audio_prompt_env.savedStoppedStreamId;
}

void audio_prompt_init_handler(void)
{
    memset((uint8_t *)&audio_prompt_env, 0, sizeof(audio_prompt_env));
    audio_prompt_set_saved_stopped_stream_id(-1);
}

bool audio_prompt_is_allow_update_volume(void)
{
    bool isAllow = true;

    uint32_t lock = int_lock_global();

    isAllow= (!(audio_prompt_env.isMixPromptOn))||
        (KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT ==
        audio_prompt_env.volume_level_override);

    int_unlock_global(lock);

    return isAllow;
}

bool audio_prompt_is_playing_ongoing(void)
{
    bool isPlayingOnGoing = false;
    uint32_t lock = int_lock_global();
    if (audio_prompt_env.isMixPromptOn)
    {
        isPlayingOnGoing = true;
    }
    int_unlock_global(lock);
    return isPlayingOnGoing;
}

#ifdef TWS_PROMPT_SYNC

#define MEDIA_SHORT_TRIGGER_DELAY       8
#define MEDIA_LONG_TRIGGER_DELAY        12

#define PROMPT_TICKS_OFFSET_TO_TRIGGER_MIX  10  // 3.25 ms

static uint32_t mix_prompt_trigger_ticks = 0;
static uint32_t playback_interval_in_ticks = 0;
static uint32_t playback_last_irq_ticks = 0;
static uint8_t isStartMixPrompt = false;

static uint32_t get_prompt_trigger_delay(uint16_t promptId)
{
    switch (promptId)
    {
        case AUDIO_ID_BT_GSOUND_MIC_OPEN:
        case AUDIO_ID_BT_GSOUND_MIC_CLOSE:
        // TODO: add more case for specific prompts here if you wanna it played sooner
            return MEDIA_SHORT_TRIGGER_DELAY;
        default:
            return MEDIA_LONG_TRIGGER_DELAY;
    }
}

bool tws_calculate_mix_prompt_trigger_ticks(uint16_t promptId)
{
    if (0 == mix_prompt_trigger_ticks)
    {
        if ((playback_interval_in_ticks > 0)&&(playback_last_irq_ticks > 0))
        {
            mix_prompt_trigger_ticks = playback_last_irq_ticks+
                get_prompt_trigger_delay(promptId)*playback_interval_in_ticks;
            //TRACE(1,"playback_last_irq_ticks %d",playback_last_irq_ticks);
            //TRACE(2,"playback_interval_in_ticks %d mix_prompt_trigger_ticks %d",playback_interval_in_ticks, mix_prompt_trigger_ticks);
            return true;
        }
    }

    return false;
}

void tws_enable_mix_prompt(bool isEnable)
{
    // TRACE(1,"isStartMixPrompt to %d.", isEnable);
    isStartMixPrompt = isEnable;
}

void tws_set_mix_prompt_trigger_ticks(uint32_t ticks)
{
    mix_prompt_trigger_ticks = ticks;
}

void tws_sync_mix_prompt_start_handling(void)
{
    if (!app_tws_ibrt_tws_link_connected())
    {
        tws_enable_mix_prompt(true);
    }
    else
    {
        tws_enable_mix_prompt(false);
    }
}

bool tws_is_mix_prompt_allowed_to_start(void)
{
    return isStartMixPrompt;
}

void app_ibrt_send_mix_prompt_req(void)
{
    if (app_tws_ibrt_tws_link_connected())
    {
        APP_TWS_CMD_MIX_PROMPT_SYNC_T req;
        req.promptId = audio_prompt_get_prompt_id();
        req.trigger_time = mix_prompt_trigger_ticks;
        req.sampleRate = audio_prompt_get_sample_rate();
        tws_ctrl_send_cmd(APP_TWS_CMD_SYNC_MIX_PROMPT_REQ,(uint8_t*)&req,sizeof(APP_TWS_CMD_MIX_PROMPT_SYNC_T));
    }
}

bool tws_sync_mix_prompt_handling(void)
{
    if (!tws_is_mix_prompt_allowed_to_start())
    {
        if (tws_calculate_mix_prompt_trigger_ticks(audio_prompt_get_prompt_id()))
        {
            // get the trigger ticks, send request to slave
            app_ibrt_send_mix_prompt_req();
        }
        return false;
    }
    else
    {
        return true;
    }
}

uint32_t tws_get_mix_prompt_trigger_ticks(void)
{
    return mix_prompt_trigger_ticks;
}

void tws_reset_mix_prompt_trigger_ticks(void)
{
    mix_prompt_trigger_ticks = 0;
    playback_interval_in_ticks = 0;
    playback_last_irq_ticks = 0;
    isStartMixPrompt = false;
}

void app_tws_stop_peer_prompt(void)
{
    uint8_t stub_param = 0;

    tws_ctrl_send_cmd(APP_TWS_CMD_STOP_PEER_PROMPT_REQ, &stub_param, sizeof(stub_param));
}

extern "C" void tws_playback_ticks_check_for_mix_prompt(void)
{
    if ((!audio_prompt_is_playing_ongoing()) || tws_is_mix_prompt_allowed_to_start())
    {
        //TRACE(0,"check for mix prompt<1>");
        return;
    }

    uint32_t curr_ticks = 0;
    uint16_t conhandle = INVALID_HANDLE;
    ibrt_ctrl_t *p_ibrt_ctrl = app_tws_ibrt_get_bt_ctrl_ctx();

    if (app_tws_ibrt_mobile_link_connected()){
        conhandle = p_ibrt_ctrl->mobile_conhandle;
        curr_ticks = bt_syn_get_curr_ticks(conhandle);
    }else if (app_tws_ibrt_slave_ibrt_link_connected()){
        conhandle = p_ibrt_ctrl->ibrt_conhandle;
        curr_ticks = bt_syn_get_curr_ticks(conhandle);
    }else{
        TRACE(0,"check for mix prompt<2>");
        return;
    }
    TRACE(1,"irq curr ticks:%d",curr_ticks);

    if (mix_prompt_trigger_ticks > 0)
    {
        if (curr_ticks >= mix_prompt_trigger_ticks)
        {
            TRACE(2,"ticks<1> %d - trigger ticks %d", curr_ticks,mix_prompt_trigger_ticks);
            tws_enable_mix_prompt(true);
        }
        else if ((curr_ticks < mix_prompt_trigger_ticks) &&
                ((mix_prompt_trigger_ticks - curr_ticks) < PROMPT_TICKS_OFFSET_TO_TRIGGER_MIX))
        {
            TRACE(2,"ticks<2> %d - trigger ticks %d", curr_ticks,mix_prompt_trigger_ticks);
            tws_enable_mix_prompt(true);
        }
    }

    if (0 != playback_last_irq_ticks)
    {
       playback_interval_in_ticks = curr_ticks-playback_last_irq_ticks;
    }

    playback_last_irq_ticks = curr_ticks;
}

void app_tws_cmd_sync_mix_prompt_req_handler(uint8_t* ptrParam, uint16_t paramLen)
{
    uint32_t curr_ticks = 0xFFFFFFFF;
    ibrt_ctrl_t *p_ibrt_ctrl = app_tws_ibrt_get_bt_ctrl_ctx();

    APP_TWS_CMD_MIX_PROMPT_SYNC_T* pReq = (APP_TWS_CMD_MIX_PROMPT_SYNC_T *)ptrParam;
    TRACE(2,"promptId, trigger_time:0x%x, %d",
        pReq->promptId, pReq->trigger_time);

    audio_prompt_stop_playing();

    if (app_tws_ibrt_slave_ibrt_link_connected())
    {
        curr_ticks = bt_syn_get_curr_ticks(p_ibrt_ctrl->ibrt_conhandle);
    }

    if ((!app_tws_ibrt_slave_ibrt_link_connected()) ||
        ((curr_ticks > pReq->trigger_time) &&
         ((curr_ticks - pReq->trigger_time) > PROMPT_TICKS_OFFSET_TO_TRIGGER_MIX)))
    {
        TRACE(0,"return directly.");
        return;
    }

    tws_set_mix_prompt_trigger_ticks(pReq->trigger_time);
    audio_prompt_start_playing(pReq->promptId, pReq->sampleRate);
    if (curr_ticks >= pReq->trigger_time)
    {
        TRACE(1,"Instant passed %d",curr_ticks);
        tws_enable_mix_prompt(true);
    }
    else if ((curr_ticks < pReq->trigger_time) &&
        ((pReq->trigger_time - curr_ticks) < PROMPT_TICKS_OFFSET_TO_TRIGGER_MIX)){
        TRACE(1,"Instant near %d",curr_ticks);
        tws_enable_mix_prompt(true);
    }
}
#endif

static int audio_prompt_resample_iter(uint8_t *buf, uint32_t len)
{
    if (!buf)
    {
        TRACE(0, "NULL pointer received in %s", __func__);
        return -1;
    }

    uint32_t leftLen = audio_prompt_env.tmpSourcePcmDataLen - audio_prompt_env.tmpSourcePcmDataOutIndex;
    uint32_t lenToFetch;

    if (leftLen >= len)
    {
        lenToFetch = len;
    }
    else
    {
        lenToFetch = leftLen;
    }

    memcpy(buf, audio_prompt_env.tmpSourcePcmDataBuf + audio_prompt_env.tmpSourcePcmDataOutIndex,
        lenToFetch);
    audio_prompt_env.tmpSourcePcmDataOutIndex += lenToFetch;

    memset(buf + lenToFetch, 0, len - lenToFetch);

    return 0;
}

void audio_prompt_buffer_config(uint8_t mixType,
                                uint8_t channel_cnt,
                                uint8_t bitNumPerSample,
                                uint8_t *tmpSourcePcmDataBuf,
                                uint8_t *tmpTargetPcmDataBuf,
                                uint8_t *pcmDataBuf,
                                uint32_t pcmBufLen,
                                uint8_t *bufForResampler,
                                uint32_t resampleBufLen)
{
    af_lock_thread();
    audio_prompt_env.mixType = mixType;
    audio_prompt_env.targetChannelCnt = channel_cnt;
    if (24 == bitNumPerSample)
    {
        audio_prompt_env.targetBytesCntPerSample = 4;
    }
    else if (16 == bitNumPerSample)
    {
        audio_prompt_env.targetBytesCntPerSample = 2;
    }
    else
    {
        ASSERT(false, "bitNumPerSample %d is not supported by prompt mixer yet!", bitNumPerSample);
    }

    audio_prompt_env.tmpSourcePcmDataBuf = tmpSourcePcmDataBuf;
    audio_prompt_env.tmpTargetPcmDataBuf = tmpTargetPcmDataBuf;
    audio_prompt_env.bufForResampler = bufForResampler;
    audio_prompt_env.resampleBufLen = resampleBufLen;

    InitCQueue(&(audio_prompt_env.pcmDataQueue), pcmBufLen, (CQItemType *)(pcmDataBuf));
    af_unlock_thread();
}


uint16_t audio_prompt_get_prompt_id(void)
{
    return audio_prompt_env.promptId;
}

uint32_t audio_prompt_get_sample_rate(void)
{
    return audio_prompt_env.targetSampleRate;
}

bool audio_prompt_start_playing(uint16_t promptId, uint32_t targetSampleRate)
{
    if (audio_prompt_is_playing_ongoing())
    {
        return false;
    }

    TRACE(0, "[%s]", __func__);

#ifdef TWS_PROMPT_SYNC
    bool isPlayingLocally = false;
    if (app_tws_ibrt_tws_link_connected())
    {
        isPlayingLocally = IS_PROMPT_PLAYED_LOCALLY(promptId);
    }
    else
    {
        isPlayingLocally = true;
    }
#endif

    promptId = PROMPT_ID_FROM_ID_VALUE(promptId);

    uint32_t lock = int_lock_global();
    audio_prompt_env.isMixPromptOn = true;

    uint8_t* promptDataPtr = NULL;
    uint32_t promptDataLen = 0;
    PROMPT_MIX_PROPERTY_T* pPromptProperty = NULL;

#ifdef MEDIA_PLAYER_SUPPORT
    media_runtime_audio_prompt_update(promptId, &promptDataPtr, &promptDataLen);
    pPromptProperty = get_prompt_mix_property(promptId);
#endif

    if (NULL == pPromptProperty)
    {
        TRACE(0,"use default mix property");

        // use default property
        audio_prompt_env.volume_level_override = DEFAULT_VOLUME_FOR_MIX_PROMPT;

        audio_prompt_env.coeff_for_mix_prompt_for_call = DEFAULT_COEFF_FOR_MIX_PROMPT_FOR_CALL;
        audio_prompt_env.coeff_for_mix_call_for_call = DEFAULT_COEFF_FOR_MIX_CALL_FOR_CALL;
        audio_prompt_env.coeff_for_mix_prompt_for_music = DEFAULT_COEFF_FOR_MIX_PROMPT_FOR_MUSIC;
        audio_prompt_env.coeff_for_mix_music_for_music = DEFAULT_COEFF_FOR_MIX_MUSIC_FOR_MUSIC;
    }
    else
    {
        audio_prompt_env.volume_level_override =
            pPromptProperty->volume_level_override;

        audio_prompt_env.coeff_for_mix_prompt_for_call =
            pPromptProperty->coeff_for_mix_prompt_for_call;
        audio_prompt_env.coeff_for_mix_call_for_call =
            pPromptProperty->coeff_for_mix_call_for_call;
        audio_prompt_env.coeff_for_mix_prompt_for_music =
            pPromptProperty->coeff_for_mix_prompt_for_music;
        audio_prompt_env.coeff_for_mix_music_for_music =
            pPromptProperty->coeff_for_mix_music_for_music;
    }

    if (KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT != audio_prompt_env.volume_level_override)
    {
        // if the prompt's volume is smaller than the current volume, don't change it
        if (audio_prompt_env.volume_level_override <= app_bt_stream_local_volume_get())
        {
            audio_prompt_env.volume_level_override = KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT;
        }
    }

    audio_prompt_env.promptId = promptId;
    audio_prompt_env.promptDataBuf = promptDataPtr;
    audio_prompt_env.isResetDecoder = true;
    audio_prompt_env.isAudioPromptDecodingDone = false;
    audio_prompt_env.wholeEncodedDataLen = promptDataLen;
    audio_prompt_env.leftEncodedDataLen = audio_prompt_env.wholeEncodedDataLen;
    audio_prompt_env.targetSampleRate = targetSampleRate;

    audio_prompt_env.resampleRatio = ((float)AUDIO_PROMPT_SBC_SAMPLE_RATE_VALUE) / audio_prompt_env.targetSampleRate;
    audio_prompt_env.targetPcmChunkSize = (uint32_t)(AUDIO_PROMPT_SBC_PCM_DATA_SIZE_PER_FRAME/audio_prompt_env.resampleRatio);

    audio_prompt_env.resampler = app_playback_resample_any_open_with_pre_allocated_buffer(( enum AUD_CHANNEL_NUM_T )AUDIO_PROMPT_SBC_CHANNEL_COUNT,
                                                                                          audio_prompt_resample_iter,
                                                                                          AUDIO_PROMPT_RESAMPLE_ITER_NUM,
                                                                                          audio_prompt_env.resampleRatio,
                                                                                          audio_prompt_env.bufForResampler,
                                                                                          audio_prompt_env.resampleBufLen);

    audio_prompt_set_saved_stopped_stream_id(-1);

    int_unlock_global(lock);

    TRACE(1,"start audio prompt. target sample rate %d", targetSampleRate);

    app_sysfreq_req(APP_SYSFREQ_USER_PROMPT_MIXER, APP_SYSFREQ_104M);

#ifdef TWS_PROMPT_SYNC
    if (!isPlayingLocally)
    {
        tws_sync_mix_prompt_start_handling();
    }
    else
    {
        tws_enable_mix_prompt(true);
    }
#endif


    return true;
}

void audio_prompt_forcefully_stop(void)
{
    app_playback_resample_close(audio_prompt_env.resampler);
    audio_prompt_set_saved_stopped_stream_id(-1);
    audio_prompt_env.isAudioPromptDecodingDone  = true;
    audio_prompt_env.leftEncodedDataLen = 0;
    app_sysfreq_req(APP_SYSFREQ_USER_PROMPT_MIXER, APP_SYSFREQ_32K);
#if defined(IBRT) && defined(MEDIA_PLAYER_SUPPORT)
    app_tws_sync_prompt_check();
#endif
}

bool audio_prompt_check_on_stopping_stream(uint8_t pendingStopOp, uint8_t deviceId)
{
    uint32_t lock = int_lock_global();
    if (audio_prompt_env.isMixPromptOn)
    {
        if (bt_is_playback_triggered())
        {
            TRACE(1,"Prompt mixing ongoing, pending op:%d", pendingStopOp);
            audio_prompt_env.pendingStopOp        = pendingStopOp;
            audio_prompt_set_saved_stopped_stream_id(deviceId);
            int_unlock_global(lock);
            return false;
        }
        else
        {
            int_unlock_global(lock);
            audio_prompt_stop_playing();
            return true;
        }
    }

    int_unlock_global(lock);
    return true;
}

bool audio_prompt_clear_pending_stream(uint8_t op)
{
    bool isToClearActiveMedia = false;

    TRACE(4,
          "%s stop_id %d pendingStopOp %d op %d",
          __func__,
          audio_prompt_get_saved_stopped_stream_id(),
          audio_prompt_env.pendingStopOp,
          op);

    if (-1 != audio_prompt_get_saved_stopped_stream_id())
    {
        uint32_t lock = int_lock_global();
        if ((PENDING_TO_STOP_A2DP_STREAMING == op) &&
            (PENDING_TO_STOP_A2DP_STREAMING == audio_prompt_get_pending_stop_op()))
        {
            audio_prompt_set_saved_stopped_stream_id(-1);
            audio_prompt_set_pending_stop_op(PENDING_TO_STOP_STREAM_INVALID);
            isToClearActiveMedia = true;
        }
        else if ((PENDING_TO_STOP_SCO_STREAMING == op) &&
                 (PENDING_TO_STOP_SCO_STREAMING == audio_prompt_get_pending_stop_op()))
        {
            audio_prompt_set_saved_stopped_stream_id(-1);
            audio_prompt_set_pending_stop_op(PENDING_TO_STOP_STREAM_INVALID);
            isToClearActiveMedia = true;
        }
        int_unlock_global(lock);
    }

    return isToClearActiveMedia;
}

void audio_prompt_stop_playing(void)
{
    if (!audio_prompt_env.isMixPromptOn)
    {
        return;
    }

    TRACE(0,"Stop audio prompt.");

    app_playback_resample_close(audio_prompt_env.resampler);

    uint32_t lock = int_lock_global();
    audio_prompt_env.leftEncodedDataLen = 0;
    audio_prompt_env.isMixPromptOn = false;

#ifdef TWS_PROMPT_SYNC
    tws_reset_mix_prompt_trigger_ticks();
#endif

    int_unlock_global(lock);
    if (KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT != audio_prompt_env.volume_level_override)
    {
        // restore the volume
        app_bt_stream_volumeset_handler(app_bt_stream_local_volume_get());
    }

    lock = int_lock_global();
    uint8_t pendingStopOp;
    int8_t savedStoppedStreamId;

    pendingStopOp = audio_prompt_env.pendingStopOp;
    savedStoppedStreamId = audio_prompt_get_saved_stopped_stream_id();

    audio_prompt_set_saved_stopped_stream_id(-1);

    int_unlock_global(lock);
    if (savedStoppedStreamId >= 0)
    {
        if (PENDING_TO_STOP_A2DP_STREAMING == pendingStopOp)
        {
            TRACE(0,"Stop the pending stopped a2dp media stream.");
            if(app_bt_is_a2dp_streaming(savedStoppedStreamId) == false){
            app_stop_a2dp_media_stream(savedStoppedStreamId);
            }
        }
        else if (PENDING_TO_STOP_SCO_STREAMING == pendingStopOp)
        {
            TRACE(0,"Stop the pending stopped sco media stream.");
            if(btapp_hfp_is_dev_sco_connected(savedStoppedStreamId) == false){
            app_stop_sco_media_stream(savedStoppedStreamId);
        }
    }
    }

    app_sysfreq_req(APP_SYSFREQ_USER_PROMPT_MIXER, APP_SYSFREQ_32K);

#if defined(IBRT) && defined(MEDIA_PLAYER_SUPPORT)
    app_tws_sync_prompt_check();
#endif
    //check if there is any pending prompt need to play
    APP_AUDIO_STATUS status_next;
    APP_AUDIO_STATUS aud_status;
    aud_status.id = APP_PLAY_BACK_AUDIO;
    if (app_audio_list_rmv_callback(&aud_status, &status_next,APP_BT_SETTING_Q_POS_HEAD, true))
    {
        TRACE(4,"%s next id: 0x%x%s, aud_id %d", __func__, status_next.id, player2str(status_next.id), status_next.aud_id);
#if defined(IBRT)
        app_ibrt_if_voice_report_handler(status_next.aud_id, true);
#endif
    }

}

static void audio_prompt_processing_handler_func(uint32_t acquiredPcmDataLen, uint8_t* pcmDataToMerge)
{
    uint32_t pcmDataToGetFromPrompt = acquiredPcmDataLen/(audio_prompt_env.targetChannelCnt*audio_prompt_env.targetBytesCntPerSample/2);

    while ((uint32_t)LengthOfCQueue(&(audio_prompt_env.pcmDataQueue)) < pcmDataToGetFromPrompt)
    {
        if (audio_prompt_env.isAudioPromptDecodingDone)
        {
            break;
        }

        // decode the audio prompt
        uint32_t returnedPcmDataLen = audio_prompt_sbc_decode(audio_prompt_env.tmpSourcePcmDataBuf,
                                                              AUDIO_PROMPT_SBC_PCM_DATA_SIZE_PER_FRAME,
                                                              audio_prompt_env.isResetDecoder);

        if (returnedPcmDataLen < AUDIO_PROMPT_SBC_PCM_DATA_SIZE_PER_FRAME)
        {
            audio_prompt_env.isAudioPromptDecodingDone = true;
        }

        audio_prompt_env.isResetDecoder = false;
        audio_prompt_env.tmpSourcePcmDataLen = returnedPcmDataLen;
        audio_prompt_env.tmpSourcePcmDataOutIndex = 0;

        // do resmpling
        if (audio_prompt_env.targetSampleRate != AUDIO_PROMPT_SBC_SAMPLE_RATE_VALUE)
        {
            uint32_t targetPcmSize = returnedPcmDataLen;
            if (AUDIO_PROMPT_SBC_PCM_DATA_SIZE_PER_FRAME == returnedPcmDataLen)
            {
                targetPcmSize = audio_prompt_env.targetPcmChunkSize;
            }
            else
            {
                targetPcmSize = (uint32_t)(returnedPcmDataLen/audio_prompt_env.resampleRatio);
            }

            targetPcmSize = (targetPcmSize/4)*4;

            app_playback_resample_run(audio_prompt_env.resampler,
                                      audio_prompt_env.tmpTargetPcmDataBuf,
                                      targetPcmSize);

            // fill into pcm data queue
            EnCQueue(&(audio_prompt_env.pcmDataQueue), audio_prompt_env.tmpTargetPcmDataBuf, targetPcmSize);
        }
        else
        {
            EnCQueue(&(audio_prompt_env.pcmDataQueue), audio_prompt_env.tmpSourcePcmDataBuf, returnedPcmDataLen);
        }
    }

    uint32_t pcmDataLenToMerge = pcmDataToGetFromPrompt;
    if ((uint32_t)LengthOfCQueue(&(audio_prompt_env.pcmDataQueue)) < pcmDataToGetFromPrompt)
    {
        pcmDataLenToMerge = LengthOfCQueue(&(audio_prompt_env.pcmDataQueue));
    }

    // copy to multiple channel if needed
    if (audio_prompt_env.targetChannelCnt > 1)
    {
        // get the data
        DeCQueue(&(audio_prompt_env.pcmDataQueue), audio_prompt_env.tmpSourcePcmDataBuf, pcmDataLenToMerge);
        app_bt_stream_copy_track_one_to_two_16bits(( int16_t * )audio_prompt_env.tmpTargetPcmDataBuf,
                                                   ( int16_t * )audio_prompt_env.tmpSourcePcmDataBuf,
                                                   pcmDataLenToMerge / sizeof(uint16_t));
    }
    else
    {
        DeCQueue(&(audio_prompt_env.pcmDataQueue), audio_prompt_env.tmpTargetPcmDataBuf, pcmDataLenToMerge);
    }


    // merge the data
    int16_t *src_buf0 = (int16_t *)audio_prompt_env.tmpTargetPcmDataBuf;
    uint32_t src_len = pcmDataLenToMerge*audio_prompt_env.targetChannelCnt/sizeof(uint16_t);

    if (2 == audio_prompt_env.targetBytesCntPerSample)
    {
        int16_t *src_buf1 = (int16_t *)pcmDataToMerge;
        int16_t *dst_buf = (int16_t *)pcmDataToMerge;
        if (MIX_WITH_A2DP_STREAMING == audio_prompt_env.mixType)
        {
            for (uint32_t i = 0; i < src_len; i++) {
                float mix_value = ( float )audio_prompt_env.coeff_for_mix_prompt_for_music * src_buf0[i] +
                                  ( float )audio_prompt_env.coeff_for_mix_music_for_music * src_buf1[i];
                if (mix_value > 32750.0) {
                    mix_value = 32750.0;
                }
                if (mix_value < -32750.0) {
                    mix_value = -32750.0;
                }
                dst_buf[i] = (int16_t)mix_value;
            }
        }
        else
        {
            for (uint32_t i = 0; i < src_len; i++) {
                float mix_value = ( float )audio_prompt_env.coeff_for_mix_prompt_for_call * src_buf0[i] +
                                  ( float )audio_prompt_env.coeff_for_mix_call_for_call * src_buf1[i];
                if (mix_value > 32750.0) {
                    mix_value = 32750.0;
                }
                if (mix_value < -32750.0) {
                    mix_value = -32750.0;
                }
                dst_buf[i] = (int16_t)mix_value;
            }
        }
    }
    else if (4 == audio_prompt_env.targetBytesCntPerSample)
    {
        int32_t *src_buf1 = (int32_t *)pcmDataToMerge;
        int32_t *dst_buf = (int32_t *)pcmDataToMerge;

        if (MIX_WITH_A2DP_STREAMING == audio_prompt_env.mixType)
        {
            for (uint32_t i = 0; i < src_len; i++) {
                float mix_value = ( float )audio_prompt_env.coeff_for_mix_prompt_for_music * (( int32_t )src_buf0[i] << 8) +
                                  ( float )audio_prompt_env.coeff_for_mix_music_for_music * src_buf1[i];

                if (mix_value > 0x7FFFF000) {
                    mix_value = 0x7FFFF000;
                }
                if (mix_value < -0x7FFFF000) {
                    mix_value = -0x7FFFF000;
                }
                dst_buf[i] = (int32_t)mix_value;
            }
        }
        else
        {
            for (uint32_t i = 0; i < src_len; i++) {
                float mix_value = ( float )audio_prompt_env.coeff_for_mix_prompt_for_call * (( int32_t )src_buf0[i] << 8) +
                                  ( float )audio_prompt_env.coeff_for_mix_prompt_for_call * src_buf1[i];

                if (mix_value > 0x7FFFF000) {
                    mix_value = 0x7FFFF000;
                }
                if (mix_value < -0x7FFFF000) {
                    mix_value = -0x7FFFF000;
                }
                dst_buf[i] = (int32_t)mix_value;
            }
        }
    }

    if (audio_prompt_env.isAudioPromptDecodingDone ||
        (0 == audio_prompt_env.leftEncodedDataLen))
    {
        // prompt playing is completed
        audio_prompt_stop_playing();
        app_sysfreq_req(APP_SYSFREQ_USER_PROMPT_MIXER, APP_SYSFREQ_32K);
    }

}

void audio_prompt_processing_handler(uint32_t acquiredPcmDataLen, uint8_t* pcmDataToMerge)
{
#ifdef TWS_PROMPT_SYNC
    if (!tws_sync_mix_prompt_handling())
    {
        return;
    }
#endif

    af_lock_thread();

    if (audio_prompt_env.leftEncodedDataLen ==
        audio_prompt_env.wholeEncodedDataLen)
    {
        if (KEEP_CURRENT_VOLUME_FOR_MIX_PROMPT != audio_prompt_env.volume_level_override)
        {
            // first entering, coordinate the volume here
            app_bt_stream_volumeset_handler(audio_prompt_env.volume_level_override);
        }
    }

    uint32_t gotDataLen = 0;
    while (gotDataLen < acquiredPcmDataLen)
    {
        uint32_t lenToGet;
        if ((acquiredPcmDataLen - gotDataLen) > AUDIO_PROMPT_PCM_FILL_UNIT_SIZE)
        {
            lenToGet = AUDIO_PROMPT_PCM_FILL_UNIT_SIZE;
        }
        else
        {
            lenToGet = acquiredPcmDataLen - gotDataLen;
        }

        audio_prompt_processing_handler_func(lenToGet, pcmDataToMerge+gotDataLen);
        gotDataLen += lenToGet;
    }
    af_unlock_thread();
}

static uint32_t audio_prompt_sbc_decode(uint8_t* pcm_buffer, uint32_t expectedOutputSize, uint8_t isReset)
{
    if (isReset)
    {
        audio_prompt_sbc_init_decoder();
    }

    uint32_t sbcDataBytesToDecode;
    unsigned int pcm_offset = 0;
    uint16_t byte_decode;
    int8_t ret;
    btif_sbc_pcm_data_t audio_prompt_PcmDecData;

get_again:
    audio_prompt_PcmDecData.data = pcm_buffer+pcm_offset;
    audio_prompt_PcmDecData.dataLen = 0;

    if (audio_prompt_env.leftEncodedDataLen > AUDIO_PROMPT_SBC_ENCODED_DATA_SIZE_PER_FRAME)
    {
        sbcDataBytesToDecode = AUDIO_PROMPT_SBC_ENCODED_DATA_SIZE_PER_FRAME;
    }
    else
    {
        sbcDataBytesToDecode = audio_prompt_env.leftEncodedDataLen;
    }

    ret = btif_sbc_decode_frames(&audio_prompt_sbc_decoder,
        audio_prompt_env.promptDataBuf+audio_prompt_env.wholeEncodedDataLen-audio_prompt_env.leftEncodedDataLen,
        sbcDataBytesToDecode, &byte_decode,
        &audio_prompt_PcmDecData, expectedOutputSize-pcm_offset,
        audio_prompt_sbc_eq_band_gain);


    audio_prompt_env.leftEncodedDataLen -= byte_decode;

    pcm_offset += audio_prompt_PcmDecData.dataLen;

    if (0 == audio_prompt_env.leftEncodedDataLen)
    {
        goto exit;
    }

    if (expectedOutputSize == pcm_offset)
    {
        goto exit;
    }

    if ((ret == BT_STS_CONTINUE) || (ret == BT_STS_SUCCESS))  {
        goto get_again;
    }

exit:
    return pcm_offset;
}
#endif // MIX_AUDIO_PROMPT_WITH_A2DP_MEDIA_ENABLED

#if defined(IBRT)
void app_tws_let_peer_device_play_audio_prompt(uint16_t promptId,
    uint8_t dev_id)
{
    app_tws_voice_prompt_to_play_t req;
    req.voicePrompt = promptId;
    req.dev_id = dev_id;
    tws_ctrl_send_cmd(APP_TWS_CMD_LET_PEER_PLAY_PROMPT, (uint8_t *)&req, sizeof(req));
}

void app_tws_cmd_prompt_play_req_handler(uint8_t *ptrParam, uint32_t paramLen)
{
#ifdef MEDIA_PLAYER_SUPPORT
    app_tws_voice_prompt_to_play_t *ptPlayReq = (app_tws_voice_prompt_to_play_t *)ptrParam;
    TRACE(2,"Receive request to play prompt 0x%x%s",
        ptPlayReq->voicePrompt, aud_id2str(ptPlayReq->voicePrompt & 0xFF));
    app_audio_manager_sendrequest(APP_BT_STREAM_MANAGER_START,
        BT_STREAM_MEDIA, ptPlayReq->dev_id, ptPlayReq->voicePrompt);
#endif
}
#endif

