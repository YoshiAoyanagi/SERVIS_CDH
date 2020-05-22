// AQU_Anomaly_handler.h

#ifndef _AQU_ANOMALY_HANDLER_h
#define _AQU_ANOMALY_HANDLER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "AppInfo.h"
#include "CommonTlmCmdPacket.h"

#define AQU_ANM_THRESHOLD_DIFF_PRESSURE		100 //kPa
#define AQU_ANM_THRESHOLD_TIME_RV_OPEN		5	//s
#define AQU_ANM_THRESHOLD_TIME_TV_OPEN		60	//s
#define AQU_ANM_THRESHOLD_TEMP				90	//degC

#define AQU_TAR_BIT_TPS_01	0b10000000
#define AQU_TAR_BIT_TPS_02	0b01000000

#define AQU_TAR_BIT_RV_01	0b10000000//0b00000001
#define AQU_TAR_BIT_RV_02	0b01000000
#define AQU_TAR_BIT_RV_03	0b00100000
#define AQU_TAR_BIT_RV_04	0b00010000

#define AQU_TAR_BIT_TEMP_RTS1	0b0000000000000001
#define AQU_TAR_BIT_TEMP_RTS2	0b0000000000000010
#define AQU_TAR_BIT_TEMP_RTS3	0b0000000000000100
#define AQU_TAR_BIT_TEMP_RTS4	0b0000000000001000
#define AQU_TAR_BIT_TEMP_VTS1	0b0000000000010000
#define AQU_TAR_BIT_TEMP_VTS2	0b0000000000100000
#define AQU_TAR_BIT_TEMP_VTS3	0b0000000001000000
#define AQU_TAR_BIT_TEMP_VTS4	0b0000000010000000
#define AQU_TAR_BIT_TEMP_VTS5	0b0000000100000000
#define AQU_TAR_BIT_TEMP_VTS6	0b0000001000000000
#define AQU_TAR_BIT_TEMP_WTS	0b0000010000000000
#define AQU_TAR_BIT_TEMP_TTS	0b0000100000000000
#define AQU_TAR_BIT_TEMP_DTS	0b0001000000000000

#define AQU_TAR_BIT_TEMP		0xFFFF

#define AQU_TAR_BIT_RTV_01	0b10000000
#define AQU_TAR_BIT_RTV_02	0b01000000
#define AQU_TAR_BIT_RTV_03	0b00100000
#define AQU_TAR_BIT_RTV_04	0b00010000

#define AQU_TEMP_SCALE	0.165873	
#define AQU_TEMP_OFFSET -259.673

#define AQU_VPS_SCALE	0.00611
#define AQU_VPS_OFFSET  -12.5

#define AQU_TPS_SCALE	0.076312	
#define AQU_TPS_OFFSET  -31.25


typedef struct
{
	unsigned int AQU_ZEUS_TIME;
	float AQU_TEMPERATURE_RTS1;
	float AQU_TEMPERATURE_RTS2;
	float AQU_TEMPERATURE_RTS3;
	float AQU_TEMPERATURE_RTS4;
	float AQU_TEMPERATURE_VTS1;
	float AQU_TEMPERATURE_VTS2;
	float AQU_TEMPERATURE_VTS3;
	float AQU_TEMPERATURE_VTS4;
	float AQU_TEMPERATURE_VTS5;
	float AQU_TEMPERATURE_VTS6;
	float AQU_TEMPERATURE_WTS;
	float AQU_TEMPERATURE_TTS;
	float AQU_TEMPERATURE_DTS;
	float AQU_PRESSURE_VPS_01;
	float AQU_PRESSURE_VPS_02;
	float AQU_PRESSURE_TPS_01;
	float AQU_PRESSURE_TPS_02;
	unsigned char AQU_VALVE_STATE_RV1;
	unsigned char AQU_VALVE_STATE_RV2;
	unsigned char AQU_VALVE_STATE_RV3;
	unsigned char AQU_VALVE_STATE_RV4;
	unsigned char AQU_VALVE_STATE_VDV;
	unsigned char AQU_VALVE_STATE_DTV1;
	unsigned char AQU_VALVE_STATE_DTV2;
	unsigned char AQU_VALVE_STATE_DTV3;
	unsigned char AQU_VALVE_STATE_DTV4;
	unsigned char AQU_VALVE_STATE_RTV1;
	unsigned char AQU_VALVE_STATE_RTV2;
	unsigned char AQU_VALVE_STATE_RTV3;
	unsigned char AQU_VALVE_STATE_RTV4;
	float AQU_OLD_PRESSURE_TPS_01;
	float AQU_OLD_PRESSURE_TPS_02;
	float AQU_DIF_PRESSURE_TPS_01;
	float AQU_DIF_PRESSURE_TPS_02;
	unsigned short AQU_VALVE_OPEN_TIME_RV1;
	unsigned short AQU_VALVE_OPEN_TIME_RV2;
	unsigned short AQU_VALVE_OPEN_TIME_RV3;
	unsigned short AQU_VALVE_OPEN_TIME_RV4;
	unsigned short AQU_VALVE_OPEN_TIME_VDV;
	unsigned short AQU_VALVE_OPEN_TIME_DTV1;
	unsigned short AQU_VALVE_OPEN_TIME_DTV2;
	unsigned short AQU_VALVE_OPEN_TIME_DTV3;
	unsigned short AQU_VALVE_OPEN_TIME_DTV4;
	unsigned short AQU_VALVE_OPEN_TIME_RTV1;
	unsigned short AQU_VALVE_OPEN_TIME_RTV2;
	unsigned short AQU_VALVE_OPEN_TIME_RTV3;
	unsigned short AQU_VALVE_OPEN_TIME_RTV4;
}AQU_TLM;



typedef enum
{
	AQU_SUCCESS,
	AQU_ANM_01,
	AQU_ANM_02,
	AQU_ANM_03,
	AQU_ANM_04,
	AQU_ANM_05,
	AQU_ANM_06,
	AQU_ANM_07,
	AQU_ANM_08,
	AQU_ANM_09,
	AQU_ANM_10,
	AQU_ANM_11,
	AQU_ANM_12,
	AQU_ANM_13,
	AQU_ANM_14,
	AQU_ANM_15,
	AQU_ANM_16,
	AQU_ANM_17,
	AQU_ANM_18,
	AQU_ANM_19,
	AQU_ANM_20,
	AQU_ANM_UNKNOWN
} AQU_ANM_STS;

typedef struct
{
	AQU_TLM		  tlm;
	unsigned char is_enabled;
	unsigned char sts;
	unsigned char  tar_tps;
	unsigned char  tar_rv1;
	unsigned char  tar_rv2;
	unsigned short tar_tmp_sens;
	unsigned char  tar_rtv_a;
	unsigned char  tar_rtv_b;
	unsigned char  tar_rtv_c;
	unsigned char  tar_rtv_d;
	float		   anm_thr_diff_ps;
	unsigned short anm_thr_time_rv_open;
	unsigned short anm_thr_time_tv_open;
	float		   anm_thr_temperature;
} AQUInfo;

extern AQUInfo aqu_info;

AppInfo AQU_An_handler(void);
void AQU_An_enable(unsigned char en);
void AQU_TlmExtract(CTCP *packet);

void AQU_An_enable(unsigned char en);
void AQU_An_set_thr_dif_pressure(float val);
void AQU_An_set_thr_temperature(float val);
void AQU_An_set_rv_open_time(unsigned short val);
void AQU_An_set_tv_open_time(unsigned short val);
#endif