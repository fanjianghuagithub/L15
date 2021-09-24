#include "hal_uart.h"
#include "hal_trace.h"
#include "string.h"
#include "hal_timer.h"
#include "cmsis_os.h"
#include "hal_key.h"
//#include "btapp.h"
#include "app_thread.h"
#include "norflash_api.h"
#include "nvrecord.h"
#include "nvrecord_dev.h"
#include "nvrecord_env.h"
//#include "app_com.h"
#include "tgt_hardware.h"
#include "app_audio.h"
#include "app_bt.h"
#include "App_bt_stream.h"
#include "apps.h"
#include "hal_spdif.h"

extern int hal_spdif_clock_out_disable(enum HAL_SPDIF_ID_T id);

#define LOG_MODULE HAL_TRACE_MODULE_APP


#define LINEIN_IRQ_GPIO FJH_AUX_DET_PIN
//extern "C" int app_play_linein_onoff(bool onoff);


static void linein_delay_timehandler(void const * param);
osTimerDef(LINEIN_LINEINDELAY, linein_delay_timehandler);
static osTimerId       linein_delay_timer = NULL;
#define LINEIN_EINT_EVT           1
#define LINEIN_IN_EINT_EVT           2
#define LINEIN_OUT_EINT_EVT           3

bool linein_mode_flg = false;

	
void linein_Send_Cmd(void);
void linein_in_Send_Cmd(void);
void linein_out_Send_Cmd(void);



const struct HAL_IOMUX_PIN_FUNCTION_MAP linein_cfg_gpio[1] =
{
    {
        LINEIN_IRQ_GPIO, HAL_IOMUX_FUNC_AS_GPIO, HAL_IOMUX_PIN_VOLTAGE_VIO, HAL_IOMUX_PIN_PULLUP_ENALBE
    },
};




void linein_irqhandler(enum HAL_GPIO_PIN_T pin)          //中断产生后执行
{
    TRACE(9,"linein_irqhandler>>>>>>>>>>>>>>>> \n");
    linein_Send_Cmd();
}




static void linein_gpio_enable_irq(enum HAL_GPIO_PIN_T pin, enum HAL_GPIO_IRQ_POLARITY_T polarity)
{
    struct HAL_GPIO_IRQ_CFG_T gpiocfg;
    hal_gpio_pin_set_dir(pin, HAL_GPIO_DIR_IN, 1);

    gpiocfg.irq_enable  = true;
    gpiocfg.irq_debounce = true;
    gpiocfg.irq_polarity = polarity;
    gpiocfg.irq_handler = linein_irqhandler;
    gpiocfg.irq_type    = HAL_GPIO_IRQ_TYPE_EDGE_SENSITIVE;

    hal_gpio_setup_irq(pin, &gpiocfg);
}




static void linein_gpio_disable_irq(enum HAL_GPIO_PIN_T pin, enum HAL_GPIO_IRQ_POLARITY_T polarity)
{
    struct HAL_GPIO_IRQ_CFG_T gpiocfg;
    hal_gpio_pin_set_dir(pin, HAL_GPIO_DIR_IN, 1);

    gpiocfg.irq_enable  = false;
    gpiocfg.irq_debounce = false;
    gpiocfg.irq_polarity = polarity;
    gpiocfg.irq_handler = NULL;
    gpiocfg.irq_type    = HAL_GPIO_IRQ_TYPE_EDGE_SENSITIVE;

    hal_gpio_setup_irq(pin, &gpiocfg);
}



static void linein_delay_timehandler(void const * param)
{

    if(hal_gpio_pin_get_val(LINEIN_IRQ_GPIO))
    {
         linein_gpio_enable_irq(LINEIN_IRQ_GPIO, HAL_GPIO_IRQ_POLARITY_LOW_FALLING);
		 if(linein_mode_flg==true)
		 {
		   linein_mode_flg = false;
		   linein_out_Send_Cmd();
		 }
		 TRACE(9,"linein_OUT>>>>>>>>>>>>>>>> \n");
	}
	else
	{
	    
			
        linein_gpio_enable_irq(LINEIN_IRQ_GPIO, HAL_GPIO_IRQ_POLARITY_HIGH_RISING);
		if(linein_mode_flg==false)
		{
		    linein_mode_flg = true;
		    linein_in_Send_Cmd();
		}
		TRACE(9,"linein_IN>>>>>>>>>>>>>>>> \n");
	}
}




void linein_delay_set_time(uint32_t millisec)
{
  if (linein_delay_timer == NULL)
  {
	 linein_delay_timer	= osTimerCreate(osTimer(LINEIN_LINEINDELAY), osTimerOnce, NULL);
	 osTimerStop(linein_delay_timer);
  } 
     osTimerStart(linein_delay_timer, millisec);
}

/************************************************************************************************/
void app_in_aux_mode(void)
{
    #ifdef AUDIO_LINEIN
	app_audio_sendrequest((uint16_t)APP_PLAY_LINEIN_AUDIO, (uint8_t)APP_BT_SETTING_OPEN, 0);
	#endif
}



extern bt_status_t LinkDisconnectDirectly(bool PowerOffFlag);
void app_deinit_bt_moudle(void)
{
  app_audio_sendrequest(APP_BT_STREAM_INVALID, (uint8_t)APP_BT_SETTING_CLOSEALL, 0);
  osDelay(500);
  LinkDisconnectDirectly(true);
  osDelay(500);
  app_bt_accessmode_set(BTIF_BAM_NOT_ACCESSIBLE);

 // app_bt_accessmode_set_req(BTIF_BAM_NOT_ACCESSIBLE);
}

extern void analog_aud_codec_nomute(void);
extern void analog_aud_codec_mute(void);
extern void app_ambmode_in(void);
/************************************************************************************************/
int app_linein_process(APP_MESSAGE_BODY * msg)
{
   if (msg->message_id == LINEIN_EINT_EVT)
   {
       linein_delay_set_time(1000);
   }
   else if(msg->message_id == LINEIN_IN_EINT_EVT)
   {
       TRACE(9,"Enter linein>>>>>>>>>>>>>>>> \n");
	   //analog_aud_codec_mute();
	   hal_gpio_pin_clr(FJH_AMP_SW_PIN);
	   //app_ambmode_in();

	   //打开line in的LDO供电
	   hal_gpio_pin_set(LDO_3V3);

		
       app_deinit_bt_moudle();
       app_in_aux_mode();
	   app_stop_10_second_timer(APP_POWEROFF_TIMER_ID);

	   app_status_indication_set(APP_STATUS_INDICATION_LINEIN_MODE);
	   osDelay(500);
	   //analog_aud_codec_nomute();
	   hal_gpio_pin_set(FJH_AMP_SW_PIN);
   }
   else if(msg->message_id == LINEIN_OUT_EINT_EVT)
   {
   	/// app_play_linein_onoff(false);//chenzhao
    //  app_bt_stream_close(APP_PLAY_LINEIN_AUDIO);
	//  osDelay(200);
	/*
	   #ifdef AUDIO_LINEIN
         app_audio_sendrequest((uint16_t)APP_PLAY_LINEIN_AUDIO, (uint8_t)APP_BT_SETTING_CLOSE, 0);
	   #endif
	   osDelay(500);
	   app_audio_sendrequest(APP_BT_STREAM_INVALID, (uint8_t)APP_BT_SETTING_OPEN, 0);
	   osDelay(500);
	   app_start_10_second_timer(APP_POWEROFF_TIMER_ID);
	   app_bt_profile_connect_manager_opening_reconnect();*/
	 // analog_aud_codec_mute();
	  hal_gpio_pin_clr(FJH_AMP_SW_PIN);
      hal_gpio_pin_clr(LDO_3V3);
	  app_shutdown(); 
   }
   return 0;
}



void linein_Send_Cmd(void)
{
  APP_MESSAGE_BLOCK msg;
  msg.mod_id			= APP_MODUAL_LINEIN_FAN;
  msg.msg_body.message_id = LINEIN_EINT_EVT;
  msg.msg_body.message_ptr = (uint32_t) NULL;
  app_mailbox_put(&msg);
}



void linein_in_Send_Cmd(void)
{
  APP_MESSAGE_BLOCK msg;
  msg.mod_id			= APP_MODUAL_LINEIN_FAN;
  msg.msg_body.message_id = LINEIN_IN_EINT_EVT;
  msg.msg_body.message_ptr = (uint32_t) NULL;
  app_mailbox_put(&msg);
}

void linein_out_Send_Cmd(void)
{
  APP_MESSAGE_BLOCK msg;
  msg.mod_id			= APP_MODUAL_LINEIN_FAN;
  msg.msg_body.message_id = LINEIN_OUT_EINT_EVT;
  msg.msg_body.message_ptr = (uint32_t) NULL;
  app_mailbox_put(&msg);
}



void linein_gpio_init(void)
{
    app_set_threadhandle(APP_MODUAL_LINEIN_FAN, app_linein_process);
    hal_iomux_init(linein_cfg_gpio, 1);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T) linein_cfg_gpio[0].pin, HAL_GPIO_DIR_IN, 1);
}


void linein_irq_en(void)
{
    linein_gpio_enable_irq(LINEIN_IRQ_GPIO, HAL_GPIO_IRQ_POLARITY_LOW_FALLING);
}



void linein_close(void)
{
   linein_gpio_disable_irq(LINEIN_IRQ_GPIO, HAL_GPIO_IRQ_POLARITY_LOW_FALLING);
}
