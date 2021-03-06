cur_dir := $(dir $(lastword $(MAKEFILE_LIST)))

src_obj := $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)*.c $(cur_dir)*.S))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)*.cpp))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)a2dp/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)avdtp/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)avctp/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)avrcp/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)conmgr/*.cpp ))
ifeq ($(AMA_VOICE),1)
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)dip/*.cpp ))
endif
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)hci/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)l2cap/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)me/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)rfcom/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)sco/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)sdp/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)spp/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)hfp/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)os/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)rfcomm/*.cpp ))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)at/*.c))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)sync/*.c))
src_obj += $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)besaud/*.cpp))

mid_obj := $(src_obj:.c=.o)
mid_obj += $(src_obj:.cpp=.o)
mid_obj += $(src_obj:.S=.o)

ifeq ($(IBRT), 1)
BTAPI_LIB_NAME := ibrt_libbt_api
else
BTAPI_LIB_NAME := $(CHIP)_libbt_api
endif

ifeq ($(HFP_1_6_ENABLE), 1)
BTAPI_LIB_NAME := $(BTAPI_LIB_NAME)_sbc_enc
endif

ifeq ($(SBC_FUNC_IN_ROM), 1)
BTAPI_LIB_NAME := $(BTAPI_LIB_NAME)_sbc_in_rom
endif

ifeq ($(BT_RF_PREFER), 2M)
BTAPI_LIB_NAME := $(BTAPI_LIB_NAME)_2m
endif

ifeq ($(BT_RF_PREFER), 3M)
BTAPI_LIB_NAME := $(BTAPI_LIB_NAME)_3m
endif

ifeq ($(BT_ONE_BRING_TWO), 1)
BTAPI_LIB_NAME := $(BTAPI_LIB_NAME)_1b2
endif

ifeq ($(BLE), 1)
BTAPI_LIB_NAME := $(BTAPI_LIB_NAME)_ble
endif
$(BTAPI_LIB_NAME)-y := $(mid_obj)

obj-y += $(BTAPI_LIB_NAME).a

ccflags-y += \
    $(BT_IF_INCLUDES) \
    $(BT_PROFILES_INCLUDES) \
    -Iservices/audio_process \
    -Iservices/overlay \
    -Iservices/nvrecord \
    -Iutils/cqueue \
    -Iutils/heap \
    -Iservices/audioflinger \
    -Iutils/lockcqueue \
    -Iservices/ble_app \
    -Iservices/ble_stack/common/api/ \
    -Iservices/bt_app \
    -Iservices/resources/ \
    -Iapps/audioplayers \
    -Iapps/main/ \
    -Iapps/key/ \
    -Iplatform/drivers/ana/ \
    -Iplatform/drivers/bt/ \
    -Iutils/intersyshci \
    -Iservices/ibrt/inc \
    -Iservices/ibrt_ui/inc \
    -Iservices/ibrt_core/inc \
	-Iservices/multimedia/audio/codec/sbc/inc \
	-Iservices/multimedia/audio/codec/sbc/src/inc \
	-Iservices/bt_app/a2dp_codecs/include

ifeq ($(A2DP_LDAC_ON),1)
ccflags-y += -Ithirdparty/audio_codec_lib/ldac/inc
endif

ifeq ($(ANC_APP),1)
CFLAGS_besaud_api.o += -DANC_APP
endif

