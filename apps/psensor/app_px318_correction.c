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
#include "app_px318_correction.h"
#include "nvrecord_env.h"
#include "apps.h"
//#include "app_com.h"
#include "norflash_api.h"
#include "nvrecord.h"
#include "nvrecord_dev.h"
#include "nvrecord_env.h"
#include "hal_bootmode.h"
#include "app_battery.h"
#include "app_thread.h"
#include "app_px318.h"
#include "factory_section.h"

#define LOG_MODULE HAL_TRACE_MODULE_APP
#define PX318J_ID 0x1C 
#define STATUS_OK 0


#define PX318J_I2C_SCL HAL_IOMUX_PIN_P2_0
#define PX318J_I2C_SDA HAL_GPIO_PIN_P2_1


/*************************************************************************************************************/
uint8_t capture_p = 0;
uint32_t PsData_com = 0;
uint16_t PsData_p =0;
/*************************************************************************************************************/

extern int I2C_WriteByte(uint8_t reg_addr, uint8_t value);
extern int I2C_WriteWord(uint8_t reg_addr, uint16_t value);
extern int I2C_ReadByte(uint8_t reg_addr, uint8_t *value);
extern int I2C_ReadWord(uint8_t reg_addr, uint16_t *value);



void px318_cmdtobox_test_timehandler(void const * param);
osTimerDef(PX318_TEST_CAPTURE, px318_cmdtobox_test_timehandler);
osTimerId       cmdtobox_test_timer = NULL;






#ifdef __FJH_P_SENSOR_CORRECTION__
struct P_SENSOR_CORRECTION_DATA{
    uint8_t PsCtGain;
	uint8_t PsCtDac;
    uint16_t PsCal;
	bool PsCorrectionFlg;
    uint16_t PsThreHigh;
	bool PsThreHighFlg;
    uint16_t PsThreLow;
	bool PsThreLowFlg;	
};


struct P_SENSOR_CORRECTION_DATA P_sensor_data_cor = {
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


void Px318j_correction_cor_init(uint8_t PsCtGain_t,uint8_t PsCtDac_t,uint16_t PsCal_t,bool PsCorrectionFlg_t,uint16_t PsThreHigh_t,bool PsThreHighFlg_t,uint16_t PsThreLow_t,bool PsThreLowFlg_t)
{
   P_sensor_data_cor.PsCtGain = PsCtGain_t;
   P_sensor_data_cor.PsCtDac = PsCtDac_t;
   P_sensor_data_cor.PsCal = PsCal_t;
   P_sensor_data_cor.PsCorrectionFlg = PsCorrectionFlg_t;
   
   P_sensor_data_cor.PsThreHigh = PsThreHigh_t;
   P_sensor_data_cor.PsThreHighFlg = PsThreHighFlg_t;
   P_sensor_data_cor.PsThreLow = PsThreLow_t;
   P_sensor_data_cor.PsThreLowFlg = PsThreLowFlg_t;   
}


uint32_t MCU_I2C_Write(uint8_t devid, uint8_t reg, uint8_t* data, uint8_t num)
{
	uint32_t ret=0;
	ret= hal_gpio_i2c_simple_send2(devid,reg,data,num);
//	TRACE("hal_gpio_i2c_simple_send ret=%d ",ret);
	return ret;
}



uint32_t MCU_I2C_Read(uint8_t devid, uint8_t reg, uint8_t* data, uint8_t num)
{
   uint32_t ret=0;
   ret = hal_gpio_i2c_simple_recv(devid,&reg,1,data,num);
   //TRACE(9,"hal_gpio_i2c_simple_send ret=%d ",ret);
   //*value=(uint16_t)buf[0]|(uint16_t)buf[1]<<8;
   return ret;
}



void PX318J_I2C_Write(uint8_t addr,uint8_t reg, uint8_t data)
{
   MCU_I2C_Write(addr, reg, &data, 1);
}



void PX318J_I2C_Write_Word(uint8_t addr, uint8_t reg, uint16_t data)
{
   uint8_t value[2];
   value[0] = data & 0x00FF;
   value[1] = data >> 8;
   MCU_I2C_Write(addr, reg, value, 2);
}



void PX318J_I2C_Read(uint8_t addr, uint8_t reg, uint8_t* data)
{
   MCU_I2C_Read(addr, reg, data, 1);
}



uint16_t PX318J_I2C_Read_Word(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t mask)
{
   uint16_t value;
   MCU_I2C_Read(addr, reg, data, 2);
   value = data[1];
   value <<= 8;
   value |= data[0];
   value &= mask;
   return value;
}





void px318j_enable(uint8_t addr, uint8_t enable)
{
  if(enable!=0)
  {
     PX318J_I2C_Write(addr, 0xF0, 0x02);
     //MCU_Delay_ms(10);
     osDelay(10); 
  }
  else
  {
     PX318J_I2C_Write(addr, 0xF0, 0x00);
     //MCU_Delay_ms(5);
     osDelay(10); 
  }
}


void testpsensorcorrection(uint8_t PsCtGain_t,uint8_t PsCtDac_t,uint16_t PsCal_t)
{
/*
	struct nvrecord_env_t *nvrecord_env;
	nv_record_env_get(&nvrecord_env);
    nvrecord_env->p_sensor.PsCtGain = PsCtGain_t;
	nvrecord_env->p_sensor.PsCtDac = PsCtDac_t;
	nvrecord_env->p_sensor.PsCal = PsCal_t;
	nvrecord_env->p_sensor.PsCorrectionflg = true;
	nv_record_env_set(nvrecord_env);
#if FPGA==0
		nv_record_flash_flush();
		norflash_flush_all_pending_op();
#endif
*/

    TRACE(9,">>>>>>>>>>>>>>>>>>>...............<<<<<<<<<<<<<<<<<<<<");
	TRACE(9,">>>>>>>>>>>>>>>>>>>...............<<<<<<<<<<<<<<<<<<<<");
	TRACE(9,"PsCtGain_t:= 0X%02x PsCtDac_t:= 0X%02x  PsCal_t:= 0X%04x \n",PsCtGain_t,PsCtDac_t,PsCal_t);		
	

	P_sensor_data_cor.PsCtGain = PsCtGain_t;
	P_sensor_data_cor.PsCtDac = PsCtDac_t;
	P_sensor_data_cor.PsCal = PsCal_t;
	P_sensor_data_cor.PsCorrectionFlg = true;


	factory_px318_data_set(P_sensor_data_cor.PsCtGain,P_sensor_data_cor.PsCtDac,P_sensor_data_cor.PsCal,P_sensor_data_cor.PsCorrectionFlg,P_sensor_data_cor.PsThreHigh,P_sensor_data_cor.PsThreHighFlg,P_sensor_data_cor.PsThreLow,P_sensor_data_cor.PsThreLowFlg);


	
 



	//......  to box succesful
	//app_gp_sensor_ldo_switch(false);
	PX318J_correction_close();




#ifdef __FJH_PX318_CUR_DELAY__
	APP_MESSAGE_BLOCK msg;
	msg.mod_id			= APP_MODUAL_PX_CUR_DELAY;
	// APP_KEY_SET_MESSAGE(uart_evt, key_code, key_event);
	msg.msg_body.message_id = (uint32_t) NULL;
	msg.msg_body.message_ptr = (uint32_t) NULL;
	app_mailbox_put(&msg);
#endif

	
	//app_battery_stop();


}



uint8_t px318j_auto_dac(uint8_t addr)
{
uint8_t testdata = 0;

uint8_t buff[5] = {0};
uint16_t PsData = 0;
bool first_data = true;
//Setting Variable
uint8_t PsCtDac = 0;
uint8_t PsDacCtrl = 0;
uint8_t PsCtGain = 0;
int16_t Dac_temp = 0;
//PI Control variable
bool PI_Control = true;
int32_t dp = 0;
int32_t di = 0;
fixedpt Kp = fixedpt_rconst(0.015);
fixedpt Ki = fixedpt_rconst(0.02);
uint8_t last_try = 0;
//Bisection method
uint8_t DacMax = 96;
uint8_t DacMin = 1;
//Sensor Initial
PX318J_I2C_Write(addr, 0x60, PsCtrl);
PX318J_I2C_Write(addr, 0x61, PsPuw);
PX318J_I2C_Write(addr, 0x62, PsPuc);
PX318J_I2C_Write(addr, 0x64, PsDrvCtrl);
//PX318J_I2C_Write(addr, 0x4F, 0x00); //WaitTime = 0
PX318J_I2C_Write(addr, 0x4F, 0x11); //WaitTime = 0
PX318J_I2C_Write(addr, 0x65, 0x01); //Reset PsDacCtrl
PX318J_I2C_Write(addr, 0x67, 0x00); //Reset PsCtDac
PX318J_I2C_Write(addr, 0x69, 0x00); //Reset PsCal
PX318J_I2C_Write(addr, 0x6A, 0x00); //Reset PsCal
PX318J_I2C_Write(addr, 0xF1, 0x01); //Close INT pin output
PX318J_I2C_Write(addr, 0xF2, 0x10); //Enable Data Ready Interrupt Halt
PX318J_I2C_Write(addr, 0xFE, 0x00); //Clear Interrupt Flag
PX318J_I2C_Write(addr, 0x80, 0x08); //Enable Fast-En(Factor function)
px318j_enable(addr, 1); //Enable Sensor


MCU_I2C_Read(addr,0x80,&testdata,1);
TRACE(9,"px318 auto_dac testdata = %d",testdata);
if(testdata!=0x08)
{
    TRACE(9,"PX318 ERROR");
    return 0;
}


PsCtGain = 0x01;
PsDacCtrl = PsCtGain;
PsCtDac = 0x00;
//First Step
while (1)
{
   if(MCU_I2C_Read(addr, 0xFE, buff, 4) != STATUS_OK) //Get Interrupt flag and PS Data.
   {
   	TRACE(9,"PX318 return 0");
   	return 0;
   }
   
   if ((buff[0] & 0x10) == 0x10) //Data Ready flag
   {
      PsData = (uint16_t)buff[2] + ((uint16_t)buff[3] << 8);
      if (first_data) //Ignore the first data.
      {
         first_data = false;
         PX318J_I2C_Write(addr, 0xFE, 0x00); //Clear Interrupt Flag
         continue;
      }
      //With last try and PS Data > 0, finish the calibration else keep going.
      if (last_try == 1 && PsData > 0)
      break;
      else
      last_try = 0;
      if (PsCtDac > 0)
      {
          //The PsCtDac is over spec, try to use the bisection method to get the right setting.
          if (PsData == 0)
          {
              DacMax = (uint8_t)PsCtDac;
              PI_Control = false;
          }
          //PS Data <= target value, finish the calibration.
          else if(PsData <= TARGET_PXY)  //
          break;
          //With the bisection method, we get the last value. finish calibration.
          else if (PsCtDac == DacMin || PsCtDac == DacMax)
          break;
     }
     //PS Data <= target value, finish the calibration.
     else if (PsData <= TARGET_PXY)
     break;
	 
     if (PI_Control) //Get the setting with PI control.
     {
         dp = PsData - TARGET_PXY;
         di += dp;
         Dac_temp = (int16_t)PsCtDac+ (int16_t)fixedpt_toint(fixedpt_xmul(Kp, fixedpt_fromint(dp))+ fixedpt_xmul(Ki, fixedpt_fromint(di)))+ (dp >= 0 ? 1 : -1);
         if (Dac_temp > 96)
         {
            if (PsCtGain == 0x0F)
            {
               last_try = 1;
               Dac_temp = 96;
            }
            else
            {
              PsCtGain = (Dac_temp >> 5) + (Dac_temp >> 4); // Dac_temp / 48
              if(PsCtGain == 0x00)
              PsCtGain = 1;
              else if (PsCtGain > 0x0F)
              PsCtGain = 0x0F;
              Dac_temp = 48;
              PsDacCtrl = (PsDacCtrl & 0xF0) | PsCtGain;
              PX318J_I2C_Write(addr, 0x65, PsDacCtrl);
            }
         }
     }
     else
     {
         if (PsData > TARGET_PXY)
         DacMin = (uint8_t)Dac_temp;
         if (PsData < PXY_FULL_RANGE && PsData > TARGET_PXY) //Reduce calculate time.
         Dac_temp += 1;
         else
         Dac_temp = (uint16_t)(DacMin + DacMax) >> 1;
     }
     PsCtDac = (uint8_t)Dac_temp;
     PX318J_I2C_Write(addr, 0x67, PsCtDac);
     PX318J_I2C_Write(addr, 0xFE, 0x00); //Clear Interrupt Flag
 }
}

TRACE(9,"PX318 middle");

px318j_enable(addr, 0); //Shutdown sensor
PX318J_I2C_Write(addr, 0xFE, 0x00); //Clear IntFlag
PX318J_I2C_Write(addr, 0xF2, 0x00); //DataHalt Disable
PX318J_I2C_Write(addr, 0x80, 0x00); //FastEn Disable
//Second Step
px318j_enable(addr, 1); //Enable sensor
uint8_t index = 0;
uint32_t Sum = 0;
do
{
   MCU_I2C_Read(addr, 0xFE, buff, 4);
   if ((buff[0] & 0x10) == 0x10)
   {
      PsData = (uint16_t)buff[2] + ((uint16_t)buff[3] << 8);
      buff[0] = 0x00;
      PX318J_I2C_Write(addr, 0xFE, 0x00);
      if(index > 1) //Ignore the first two data
      Sum += PsData;
      index++;
    }
}while (index < 10);
    px318j_enable(addr, 0); //Shutdown sensor
    //Write Calibration
    PsData = (uint16_t)(Sum >> 3) + 20;
    PX318J_I2C_Write_Word(addr, 0x69, PsData);


    testpsensorcorrection(PsCtGain,PsCtDac,PsData);
	
    return 1;
}








uint32_t PX318J_correction_close(void)
{
   hal_gpio_i2c_close();
   return 0;
}



uint32_t PX318J_correction_open(void)
{
	static const struct HAL_GPIO_I2C_CONFIG_T i2c_cfg={PX318J_I2C_SCL ,PX318J_I2C_SDA,20};


    //test();


	
	hal_gpio_i2c_open(&i2c_cfg);

    px318j_auto_dac(PX318J_ID);
		
    return 0;
}



/*************************************************************************************************************/

uint8_t PX318J_correction_init(void) {

	uint8_t testdataok=0;

	//struct nvrecord_env_t *nvrecord_env;
	//nv_record_env_get(&nvrecord_env);
	
	I2C_WriteByte( 0xF4, 0xEE); //软重启	
	osDelay(40); //软重启后必要的延迟 30 毫秒	
	I2C_WriteByte( 0x60, 0x15); //PsData 为 10 位无号数，反应时间为 30 毫秒。	15:平均2次，时间为40毫秒
	//I2C_WriteByte( 0x61, 0x10); //脉冲宽度为 32W = 64 微秒	
	I2C_WriteByte( 0x61, 0x0a); //脉冲宽度为 32W = 64 微秒	
	I2C_WriteByte( 0x64, 0x0B); //驱动电流为 12 毫安	
	I2C_WriteByte( 0x4f, 0x11); 
	//I2C_WriteByte( 0x6B, 0x15);//中断产生算法 迟滞中断，5次后触发 

/*
#ifdef __FJH_P_SENSOR_CORRECTION__
if(P_sensor_data.PsCorrectionFlg)
{
	I2C_WriteByte( 0x65, P_sensor_data.PsCtGain);    // 
	I2C_WriteByte( 0x67, P_sensor_data.PsCtDac);     // 
	I2C_WriteByte( 0x69, ((uint8_t)(P_sensor_data.PsCal&0x00ff))); // 
	I2C_WriteByte( 0x6A, ((uint8_t)(P_sensor_data.PsCal>>8)));     // 	
	//0x69reg write: 0x100 6Areg write: 0x10
}
else  //默认值...
{
  
	I2C_WriteByte( 0x65, 0x01);     
	I2C_WriteByte( 0x67, 0x01);      
	I2C_WriteByte( 0x69, 0x98);  
	I2C_WriteByte( 0x6A, 0x00);      	
}
#endif
*/
    if(P_sensor_data_cor.PsCorrectionFlg==true)
    {
	    I2C_WriteByte( 0x65, P_sensor_data_cor.PsCtGain);                   // 
	    I2C_WriteByte( 0x67, P_sensor_data_cor.PsCtDac);                    // 
	    I2C_WriteByte( 0x69, ((uint8_t)(P_sensor_data_cor.PsCal&0x00ff))); // 
	    I2C_WriteByte( 0x6A, ((uint8_t)(P_sensor_data_cor.PsCal>>8)));     // 	
	}
	else
	{
		I2C_WriteByte( 0x65, 0x01); 	
		I2C_WriteByte( 0x67, 0);//0x01); 	 
		I2C_WriteByte( 0x69, 0x53);//0x98);  
		I2C_WriteByte( 0x6A, 0x00);   //chenzhao 
	}



	I2C_WriteByte( 0xF1, 0x00); //关闭中断功能

	
	//I2C_WriteWord( 0x6C, PX318_LOW_THRESHOLD); //设定低阀值 
	//I2C_WriteWord( 0x6E, PX318_HIGH_THRESHOLD); //设定高阀值 


	
	//I2C_WriteByte( 0xFE, 0x00); //清除中断旗标，主要是清除 POR 旗标。	
	//I2C_WriteByte( 0xF1, 0x03);	
	I2C_WriteByte( 0xF0, 0x02); //传感器启动	
	osDelay(20); //启动后必要的延迟 10 毫秒 
	//I2C_WriteByte( 0xFE, 0x00); 
	I2C_ReadByte( 0xF0, &testdataok);
	//TRACE(9,"px318 testdata = %d",testdata);
    return testdataok;

}



uint16_t px318_get_PsData_val()
{
	while(1)
	{
	  osDelay(30);
	  I2C_ReadWord(0x00, &PsData_p);    //读取传感器数据	
	  TRACE(9,"PsData = 0x%04x	 num_p = %d",PsData_p,capture_p);
	  PsData_com +=PsData_p;
	  capture_p++; 
	  if(capture_p==10)
	  {
		 PsData_p = PsData_com/10;
         return PsData_p;
	  }	  
	  
	}
}


void PX318J_correction_earin(void)
{
  //  struct nvrecord_env_t *nvrecord_env;
	static const struct HAL_GPIO_I2C_CONFIG_T i2c_cfg={PX318J_I2C_SCL ,PX318J_I2C_SDA,20};
	
	//app_gp_sensor_ldo_switch(false);
	//osDelay(500);

    //test();

	

	
	hal_gpio_i2c_open(&i2c_cfg);
	if(PX318J_correction_init()!=0x02)
	{
       TRACE(9,"PX318J_correction_earin ERROR");
	   return;
	}
	PsData_com = 0;
    capture_p = 0;
	px318_get_PsData_val();

	TRACE(9,"Earin PsData = 0x%04x",PsData_p);
	/*
	nv_record_env_get(&nvrecord_env);
    nvrecord_env->p_sensor.PsThreHigh_P = PsData_p;
	nvrecord_env->p_sensor.PsThreHighflg = true;
	nv_record_env_set(nvrecord_env);

#if FPGA==0
	nv_record_flash_flush();
	norflash_flush_all_pending_op();
#endif*/

	P_sensor_data_cor.PsThreHigh = PsData_p;
	P_sensor_data_cor.PsThreHighFlg = true;

	factory_px318_data_set(P_sensor_data_cor.PsCtGain,P_sensor_data_cor.PsCtDac,P_sensor_data_cor.PsCal,P_sensor_data_cor.PsCorrectionFlg,P_sensor_data_cor.PsThreHigh,P_sensor_data_cor.PsThreHighFlg,P_sensor_data_cor.PsThreLow,P_sensor_data_cor.PsThreLowFlg);




	
	


	//......  to box succesful
	//app_gp_sensor_ldo_switch(false);
	PX318J_correction_close();


#ifdef __FJH_PX318_CUR_DELAY__
	APP_MESSAGE_BLOCK msg;
	msg.mod_id			= APP_MODUAL_PX_CUR_DELAY;
	// APP_KEY_SET_MESSAGE(uart_evt, key_code, key_event);
	msg.msg_body.message_id = (uint32_t) NULL;
	msg.msg_body.message_ptr = (uint32_t) NULL;
	app_mailbox_put(&msg);
#endif
	
}


void PX318J_correction_earout(void)
{
   // struct nvrecord_env_t *nvrecord_env;

	static const struct HAL_GPIO_I2C_CONFIG_T i2c_cfg={PX318J_I2C_SCL ,PX318J_I2C_SDA,20};
	//app_gp_sensor_ldo_switch(false);
	//osDelay(500);	
	
    //test();
	

	
	hal_gpio_i2c_open(&i2c_cfg);
	if(PX318J_correction_init()!=0x02)
	{
       TRACE(9,"PX318J_correction_earout ERROR");
	   return;
	}

	PsData_com = 0;
    capture_p = 0;
	px318_get_PsData_val();
	TRACE(9,"Earout PsData = 0x%04x",PsData_p);

/*
	nv_record_env_get(&nvrecord_env);
     nvrecord_env->p_sensor.PsThreLow_p = PsData_p;
	nvrecord_env->p_sensor.PsThreLowflg = true;
	nv_record_env_set(nvrecord_env);

#if FPGA==0
	nv_record_flash_flush();
	norflash_flush_all_pending_op();
#endif*/

	P_sensor_data_cor.PsThreLow = PsData_p;
	P_sensor_data_cor.PsThreLowFlg = true; 

	factory_px318_data_set(P_sensor_data_cor.PsCtGain,P_sensor_data_cor.PsCtDac,P_sensor_data_cor.PsCal,P_sensor_data_cor.PsCorrectionFlg,P_sensor_data_cor.PsThreHigh,P_sensor_data_cor.PsThreHighFlg,P_sensor_data_cor.PsThreLow,P_sensor_data_cor.PsThreLowFlg);

	
    //app_voice_report(APP_STATUS_INDICATION_DU, 0);

	//......  to box succesful
	//app_gp_sensor_ldo_switch(false);
	PX318J_correction_close();


#ifdef __FJH_PX318_CUR_DELAY__
	APP_MESSAGE_BLOCK msg;
	msg.mod_id			= APP_MODUAL_PX_CUR_DELAY;
	// APP_KEY_SET_MESSAGE(uart_evt, key_code, key_event);
	msg.msg_body.message_id = (uint32_t) NULL;
	msg.msg_body.message_ptr = (uint32_t) NULL;
	app_mailbox_put(&msg);
#endif

}

/*************************************************************************************************************/





#ifdef __FJH_PX318_CUR_DELAY__
void px318_cmdtobox_test_timehandler(void const * param)
{
   //uint8_t testpsensorcmd = 0xDD;
   //app_uart_send2box(&testpsensorcmd,1);
   //osDelay(1000);	
   //复位....
   //hal_sw_bootmode_clear(HAL_SW_BOOTMODE_REBOOT);
   //hal_sw_bootmode_set(HAL_SW_BOOTMODE_REBOOT_CHARINGPLUG_OUT);
   //app_reset();	
   //px318_get_PsData_val();

}



int app_px318_cur_process(APP_MESSAGE_BODY * msg)
{
   send_key_event(HAL_KEY_CODE_FN5, HAL_KEY_EVENT_PSENSOR_COM); 
   TRACE(9,"send_key_event(HAL_KEY_CODE_PWR, HAL_KEY_EVENT_PSENSOR_COM);");
   return 0;


   if (cmdtobox_test_timer == NULL)
   {
	   cmdtobox_test_timer = osTimerCreate(osTimer(PX318_TEST_CAPTURE), osTimerPeriodic, NULL);
	   osTimerStop(cmdtobox_test_timer);
   }
   
   osTimerStart(cmdtobox_test_timer, 100); 
   return 0;
}




void PX318J_correction_threadhandle_open(void)
{
	app_set_threadhandle(APP_MODUAL_PX_CUR_DELAY, app_px318_cur_process);
}
#endif





