// Temperature.h

#ifndef _TEMPERATURE_h
#define _TEMPERATURE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Config.h"
#include "Utility.h"
#include "port_config.h"

#define	TEMP_TASK_INTERVAL			5

#define TEMP_OFFSET					3.29		// Board#2 ADG706 On resistance 
#define TEMP_GAIN					11.0176		//amp gain

#define TEMP_GAIN_CONV_8BIT			1	//0degC‚Å128dn, 1degC/1dn
#define TEMP_OFFSET_CONV_8BIT		128	//0degC‚Å128dn, 1degC/1dn

#define TEMP_CAL_FACTOR				1
#define TEMP_CAL_BIAS				0

#include "AppInfo.h"

typedef struct
{
	AN_TLM an_tlm[16];
}TempSens;

extern TempSens temp_sens;
AppInfo temperature_update(void);


void TempSens_init(void);
void TempSens_select(unsigned char ch);
float TempSens_channel_read(unsigned char ch);
float TempSens_Calibration(float temp);
void TempSens_Task(void);
void TempSens_DebugMessage(void);
#endif

