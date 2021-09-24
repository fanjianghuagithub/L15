/*
 
 
*/


#include "plat_types.h"
#include "hal_i2c.h"
#include "hal_i2s.h"
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


#define LOG_MODULE HAL_TRACE_MODULE_APP
uint8_t testdata=0;
uint8_t testdata_2=0;//chenzhao

#ifdef __FJH_I2C_PSENSOR__
bool PsenserEarInFlg = false;
#endif


bool px318interror =false;

uint8_t IntFlag=0;
uint16_t PsData;
void PX318J_init(void);
void PX318J_gpio_init(void);

#define PX318J_ID 0x1C 

//#define PX318J_IRQ_GPIO HAL_GPIO_PIN_P0_0
//#define PX318J_I2C_SCL HAL_IOMUX_PIN_P0_2
//#define PX318J_I2C_SDA HAL_IOMUX_PIN_P0_3

#define PX318J_IRQ_GPIO HAL_IOMUX_PIN_P2_5
#define PX318J_I2C_SCL HAL_IOMUX_PIN_P2_0
#define PX318J_I2C_SDA HAL_GPIO_PIN_P2_1



#define PX318_HIGH_THRESHOLD 0x9f///0x6f//0x96//0x138//0x168/0x6f//0x100///0x100//0x210//0x6f//0x160//0x344
#define PX318_LOW_THRESHOLD 0x2f//0x6f//0x95//0xb0//0x100//0x4f//0x7f//0x30//0x100//0x218//chenzhao changed







#ifdef __FJH_GSENSOR_DELAY_OPEN__
static void gensor_delay_timehandler(void const * param);
osTimerDef(PSENSOROPEN_GENSORDELAY, gensor_delay_timehandler);
static osTimerId       gensor_delay_timer = NULL;
#define GENSOR_EINT_EVT           1
void Gensor_Send_Cmd(void);
#endif

bool GensorEnableFlg;


const struct HAL_IOMUX_PIN_FUNCTION_MAP PX318J_cfg_gpio[1] =
{
    {
        PX318J_IRQ_GPIO, HAL_IOMUX_FUNC_AS_GPIO, HAL_IOMUX_PIN_VOLTAGE_VIO, HAL_IOMUX_PIN_NOPULL
    },
};



#ifdef __FJH_P_SENSOR_CORRECTION__
struct P_SENSOR_DATA{
    uint8_t PsCtGain;
	uint8_t PsCtDac;
    uint16_t PsCal;
	bool PsCorrectionFlg;
    uint16_t PsThreHigh;
	bool PsThreHighFlg;
    uint16_t PsThreLow;
	bool PsThreLowFlg;

	
};


struct P_SENSOR_DATA P_sensor_data = {
  .PsCtGain = 0,
  .PsCtDac = 0,
  .PsCal = 0,
  .PsCorrectionFlg = false,
  .PsThreHigh = 0,
  .PsThreHighFlg = false,
  .PsThreLow = 0,
  .PsThreLowFlg = false
};
#endif


bool testearinflg = false;
#ifdef __FJH_GSENSOR_DELAY_OPEN__
static void gensor_delay_timehandler(void const * param)
{
#if 0
   if(PsenserEarInFlg == false)
   {
     TRACE("gensor_delay_timehandler>>>PsenserEarInFlg == false \n");
     return;
   }
#endif   
   if(testearinflg == false)
   {
      if(PsenserEarInFlg==true)
      {
        GensorEnableFlg = true;
		testearinflg = true;
        send_key_event(HAL_KEY_CODE_FN5, HAL_KEY_EVENT_PSENSOR_EARIN);	
        TRACE(9,"send_key_event(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_PSENSOR_EARIN)\n;");
      }
   }
   else
   {
      if(PsenserEarInFlg==false)
      {
       GensorEnableFlg = false;
	   testearinflg = false;
	   send_key_event(HAL_KEY_CODE_FN5, HAL_KEY_EVENT_PSENSOR_EAROUT);	  
	   TRACE(9,"send_key_event(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_PSENSOR_EAROUT);");
      }
   }

   
   TRACE(9,"gensor_delay_timehandler>>>>>>>>>>>>>>>> \n");
}



void gensor_delay_set_time(uint32_t millisec)
{
 osStatus tt;
  if (gensor_delay_timer == NULL)
  {
	 gensor_delay_timer	= osTimerCreate(osTimer(PSENSOROPEN_GENSORDELAY), osTimerOnce, NULL);
	 osTimerStop(gensor_delay_timer);
  } 
      tt = osTimerStart(gensor_delay_timer, millisec);
	  TRACE(9,"osTimerStart:= %08x \n",tt);
}
#endif




/* i2c write */
int I2C_WriteByte(uint8_t reg_addr, uint8_t value)
{
	uint8_t buf[2];
    buf[0] = reg_addr;
    buf[1] = value;
	uint32_t ret=0;
	ret= hal_gpio_i2c_simple_send(PX318J_ID,buf,2);
//	TRACE("hal_gpio_i2c_simple_send ret=%d ",ret);
	return ret;
}

int I2C_WriteWord(uint8_t reg_addr, uint16_t value)
{

	uint8_t buf[3];
    buf[0] = reg_addr;
    //buf[1] = (uint8_t)(value>>8);
    //buf[2] = (uint8_t)(value&0xff);
    buf[1] = (uint8_t)(value&0xff);
    buf[2] = (uint8_t)(value>>8);

	
	uint32_t ret=0;
	ret= hal_gpio_i2c_simple_send(PX318J_ID,buf,3);
	//TRACE("hal_gpio_i2c_simple_send ret=%d ",ret);
	return ret;

}
int I2C_ReadByte(uint8_t reg_addr, uint8_t *value)
{
	//uint8_t buf[2]={0,0};
	uint32_t ret=0;
	ret= hal_gpio_i2c_simple_recv(PX318J_ID,&reg_addr,1,value,1);
	//TRACE(9,"hal_gpio_i2c_simple_send ret=%d ",ret);
	
	return ret;

}

int I2C_ReadWord(uint8_t reg_addr, uint16_t *value)
{
	uint8_t buf[2]={0,0};
	uint32_t ret=0;
	ret= hal_gpio_i2c_simple_recv(PX318J_ID,&reg_addr,1,buf,2);
	//TRACE(9,"hal_gpio_i2c_simple_send ret=%d ",ret);
	*value=(uint16_t)buf[0]|(uint16_t)buf[1]<<8;
	
	return ret;

}




void PX318J_init(void) {


	
	I2C_WriteByte( 0xF4, 0xEE); //软重启	
	osDelay(40); //软重启后必要的延迟 30 毫秒	
	I2C_WriteByte( 0x60, 0x15); //PsData 为 10 位无号数，反应时间为 30 毫秒。	15:平均2次，时间为40毫秒
	//I2C_WriteByte( 0x61, 0x10); //脉冲宽度为 32W = 64 微秒	
	I2C_WriteByte( 0x61, 0x0a); //脉冲宽度为 32W = 64 微秒	
	I2C_WriteByte( 0x64, 0x0B); //驱动电流为 12 毫安	
	I2C_WriteByte( 0x4f, 0x11); 
	//I2C_WriteByte( 0x6B, 0x15);//中断产生算法 迟滞中断，5次后触发 


#ifdef __FJH_P_SENSOR_CORRECTION__


if(0)///P_sensor_data.PsCorrectionFlg)
{
	I2C_WriteByte( 0x65, P_sensor_data.PsCtGain);    // 
	I2C_WriteByte( 0x67, P_sensor_data.PsCtDac);     // 
	I2C_WriteByte( 0x69, ((uint8_t)(P_sensor_data.PsCal&0x00ff))); // 
	I2C_WriteByte( 0x6A, ((uint8_t)(P_sensor_data.PsCal>>8)));     // 	
	//0x69reg write: 0x100 6Areg write: 0x10
}
else  //默认值...
{
  
	/*I2C_WriteByte( 0x65, 0x0a);     
	I2C_WriteByte( 0x67, 0x26);      
	I2C_WriteByte( 0x69, 0x86);  
	I2C_WriteByte( 0x6A, 0x00);      	*/

	/*I2C_WriteByte( 0x65, 0x01);     
	I2C_WriteByte( 0x67, 0x00);      
	I2C_WriteByte( 0x69, 0x6f);//0x53); //0x14);  
	I2C_WriteByte( 0x6A, 0x00);//0x00);*///chenzhao type 2
	
	I2C_WriteByte( 0x65, 0x01);//0x01);     
	I2C_WriteByte( 0x67, 0x48);//0x00);      
	I2C_WriteByte( 0x69, 0x7f);//0x53); //0x14);  
	I2C_WriteByte( 0x6A, 0x00);//0x00);//chenzhao type 1

	//TRACE(0,"P_SENSOR_CORRECTION I2C_WriteByte 0x02");
}
#endif

	

    if(P_sensor_data.PsThreLowFlg)	
    {
       I2C_WriteWord( 0x6C, P_sensor_data.PsThreLow);   //设定低阀值 
	}
	else
	{
	   I2C_WriteWord( 0x6C, PX318_LOW_THRESHOLD);      //设定低阀值 
	}
	if(P_sensor_data.PsThreHighFlg)
	{

       I2C_WriteWord( 0x6E, P_sensor_data.PsThreHigh); //设定高阀值 
	}
	else
	{
	   I2C_WriteWord( 0x6E, PX318_HIGH_THRESHOLD);     //设定高阀值 
	}

	
	I2C_WriteByte( 0xFE, 0x00); //清除中断旗标，主要是清除 POR 旗标。	
	//I2C_WriteByte( 0xF1, 0x03);	
	I2C_WriteByte( 0xF0, 0x02); //传感器启动	
	osDelay(20); //启动后必要的延迟 10 毫秒 
	//I2C_WriteByte( 0xFE, 0x00); 

	I2C_ReadByte( 0xF0, &testdata);
	TRACE(9,"px318 testdata = %d",testdata);

	#if 0//chenzhao
	I2C_ReadByte(0x4f, &testdata_2);
	TRACE(1,"px318 0x4f testdata = %d",testdata_2);
	#endif



//*PsCtGain_t:= 0X09 PsCtDac_t:= 0X0c  PsCal_t:= 0X0023 PsCorrectionflg:= 1 

//PsThreHigh_P:= 0X0358 PsThreHighflg:= 1 PsThreLow_p:= 0X00b9 PsThreLowflg:= 1 


	
	
}



void PX318J_get_val()
{
	I2C_ReadWord(0x00, &PsData); //读取传感器数据	
	TRACE(9,"px318 0x00  data=0x%x ",PsData);
	I2C_ReadByte( 0xFF, &testdata); //读取中断旗标状态	
	TRACE(9,"px318 testdata = 0x%x",testdata);		
}






void PX318J_irqhandler(enum HAL_GPIO_PIN_T pin) //中断产生后执行
{
	I2C_ReadByte( 0xFE, &IntFlag); //读取中断旗标状态	
	TRACE(9,"px318 IntFlag = 0x%x",IntFlag);	
	I2C_ReadByte( 0xFF, &testdata); //读取中断旗标状态	
	TRACE(9,"px318 irq testdata = 0x%x",testdata);	

	TRACE(0,"int flaggggggggggggggggg");

	if(IntFlag & 0x02) 		
		//TRACE("int flaggggggggggggggggg");	
	{ 
		//确认中断来自于传感器 PsInt	
		I2C_ReadWord(0x00, &PsData); //读取传感器数据	
		TRACE(9,"px318 data=0x%x IntFlag = 0x%x\n",PsData,IntFlag);
		I2C_WriteByte( 0xFE, 0x00);  //透过写入 0x00 至寄存器 0xFE 来清除中断旗标	


        if(px318interror == true)
			return;

		
        //进入中断表示检测到了入耳/出耳
        if(PsData>PX318_HIGH_THRESHOLD)       //入耳
        {
		
#ifdef __FJH_GSENSOR_DELAY_OPEN__
		 Gensor_Send_Cmd();
#endif


		
#ifdef __FJH_I2C_PSENSOR__
			PsenserEarInFlg = true;
		//	send_key_event_apps(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_PSENSOR_EARIN);  
		//	TRACE("send_key_event(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_PSENSOR_EARIN);");
#endif

		}
		else if(PsData<PX318_LOW_THRESHOLD)   //出耳
		{
		
#ifdef __FJH_GSENSOR_DELAY_OPEN__
		    Gensor_Send_Cmd();
#endif

            GensorEnableFlg = false;
		
#ifdef __FJH_I2C_PSENSOR__
            PsenserEarInFlg = false;
			//send_key_event_apps(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_PSENSOR_EAROUT); 
			//TRACE("send_key_event(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_PSENSOR_EAROUT);");
#endif
		}	
	}

}


static void PX318J_gpio_enable_irq(enum HAL_GPIO_PIN_T pin, enum HAL_GPIO_IRQ_POLARITY_T polarity)
{
    struct HAL_GPIO_IRQ_CFG_T gpiocfg;
    hal_gpio_pin_set_dir(pin, HAL_GPIO_DIR_IN, 1);

    gpiocfg.irq_enable  = true;
    gpiocfg.irq_debounce = true;
    gpiocfg.irq_polarity = polarity;
    gpiocfg.irq_handler = PX318J_irqhandler;
    gpiocfg.irq_type    = HAL_GPIO_IRQ_TYPE_EDGE_SENSITIVE;

    hal_gpio_setup_irq(pin, &gpiocfg);
}



static void PX318J_gpio_disable_irq(enum HAL_GPIO_PIN_T pin, enum HAL_GPIO_IRQ_POLARITY_T polarity)
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


#ifdef __FJH_P_SENSOR_CORRECTION__
void Px318j_correction_init(uint8_t PsCtGain_t,uint8_t PsCtDac_t,uint16_t PsCal_t,bool PsCorrectionFlg_t,uint16_t PsThreHigh_t,bool PsThreHighFlg_t,uint16_t PsThreLow_t,bool PsThreLowFlg_t)
{
   P_sensor_data.PsCtGain = PsCtGain_t;
   P_sensor_data.PsCtDac = PsCtDac_t;
   P_sensor_data.PsCal = PsCal_t;
   P_sensor_data.PsCorrectionFlg = PsCorrectionFlg_t;
   
   P_sensor_data.PsThreHigh = PsThreHigh_t;
   P_sensor_data.PsThreHighFlg = PsThreHighFlg_t;
   P_sensor_data.PsThreLow = PsThreLow_t;
   P_sensor_data.PsThreLowFlg = PsThreLowFlg_t;   
}
#endif







#ifdef __FJH_GSENSOR_DELAY_OPEN__
void Gensor_Send_Cmd(void)
{
  APP_MESSAGE_BLOCK msg;
  msg.mod_id			= APP_MODUAL_GENSOR_DELAY;
  msg.msg_body.message_id = GENSOR_EINT_EVT;
  msg.msg_body.message_ptr = (uint32_t) NULL;
  app_mailbox_put(&msg);
}




int app_gensor_process(APP_MESSAGE_BODY * msg)
{
   if (msg->message_id == GENSOR_EINT_EVT)
   {
       gensor_delay_set_time(700);//1300);///700);//chenzhao
   }
   return 0;
}
#endif


void PX318J_gpio_init(void)
{
    hal_iomux_init(PX318J_cfg_gpio, 1);
    hal_gpio_pin_set_dir((enum HAL_GPIO_PIN_T) PX318J_cfg_gpio[0].pin, HAL_GPIO_DIR_IN, 1);

    PX318J_gpio_enable_irq(PX318J_IRQ_GPIO, HAL_GPIO_IRQ_POLARITY_LOW_FALLING);
}





uint32_t PX318J_close(void)
{
return 0;
   PX318J_gpio_disable_irq(PX318J_IRQ_GPIO, HAL_GPIO_IRQ_POLARITY_LOW_FALLING);
   hal_gpio_i2c_close();

   
}


struct HAL_IOMUX_PIN_FUNCTION_MAP hal_gpio_i2c_iomux_test[] = {
	{PX318J_I2C_SCL, HAL_IOMUX_FUNC_GPIO, HAL_IOMUX_PIN_VOLTAGE_VIO, HAL_IOMUX_PIN_NOPULL},
	{PX318J_I2C_SDA, HAL_IOMUX_FUNC_GPIO, HAL_IOMUX_PIN_VOLTAGE_VIO, HAL_IOMUX_PIN_NOPULL},
};


uint32_t PX318J_open(void)
{
return 0;
    static const struct HAL_GPIO_I2C_CONFIG_T i2c_cfg={PX318J_I2C_SCL,PX318J_I2C_SDA,20};
	/*
    hal_iomux_init((struct HAL_IOMUX_PIN_FUNCTION_MAP *)hal_gpio_i2c_iomux_test, sizeof(hal_gpio_i2c_iomux_test)/sizeof(struct HAL_IOMUX_PIN_FUNCTION_MAP));
	TRACE(9,"PX318J_open1111111111111111");
	
	hal_gpio_pin_set_dir(PX318J_I2C_SCL, HAL_GPIO_DIR_OUT, 0);
    hal_gpio_pin_set_dir(PX318J_I2C_SDA, HAL_GPIO_DIR_OUT, 0);

	return 0;
    */


 
	hal_gpio_i2c_open(&i2c_cfg);

	TRACE(9,"PX318J_open1111111111111111");


#ifdef __FJH_GSENSOR_DELAY_OPEN__
	app_set_threadhandle(APP_MODUAL_GENSOR_DELAY, app_gensor_process);
#endif

	PX318J_gpio_init();
    PX318J_init();

    return 0;
}

void PX318J_open_cz(void)//chenzhao
{
	//uint32_t lock;

    //lock = int_lock();
	///TRACE(9,"PX318J_open22222222222");

	PX318J_gpio_init();
    PX318J_init();
	
	//int_unlock(lock);
    //return 0;
}

