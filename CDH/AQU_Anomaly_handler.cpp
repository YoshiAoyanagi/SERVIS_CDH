// 
// 
// 

#include "AQU_Anomaly_handler.h"


#include <stddef.h> // for NULL
#include <string.h> // for memcpy

#include "cmd_analyze_obc.h"
#include "BlockCommandDefinisions.h"
#include "utility.h"

AQUInfo aqu_info;

static void AQU_An_init_(void);
static void AQU_An_execute_(void);
static void AQU_Anm_detect_func(unsigned short tar, unsigned short tar_bit, float val, float thr, unsigned char anm_sts);
void AQU_Anm_detect_time_func(unsigned short tar, unsigned short tar_bit, unsigned char sts, unsigned short thr,
	unsigned char anm_sts, unsigned short *times);

void AQU_Anm_detect_open_time_func(unsigned short tar, unsigned short tar_bit, unsigned short thr, unsigned char anm_sts, unsigned short times);

AppInfo AQU_An_handler(void)
{
	return create_app_info("aqu", 5, AQU_An_init_, AQU_An_execute_);
}

static void AQU_An_init_(void)
{
	aqu_info.is_enabled = 0;
	aqu_info.sts = AQU_SUCCESS;
	aqu_info.anm_thr_diff_ps		= AQU_ANM_THRESHOLD_DIFF_PRESSURE;
	aqu_info.anm_thr_time_rv_open	= AQU_ANM_THRESHOLD_TIME_RV_OPEN;
	aqu_info.anm_thr_time_tv_open	= AQU_ANM_THRESHOLD_TIME_TV_OPEN;
	aqu_info.anm_thr_temperature	= AQU_ANM_THRESHOLD_TEMP;

	aqu_info.tar_tps	= 0;
	aqu_info.tar_rv1	= 0;
	aqu_info.tar_rv2	= 0;
	aqu_info.tar_tmp_sens = 0xFFFF;
	aqu_info.tar_rtv_a	= 0;
	aqu_info.tar_rtv_b = 0;
	aqu_info.tar_rtv_c = 0;
	aqu_info.tar_rtv_d = 0;
}

void AQU_An_enable(unsigned char en)
{
	aqu_info.is_enabled = en;
}

void AQU_An_set_thr_dif_pressure(float val)
{
	aqu_info.anm_thr_diff_ps = val;
}

void AQU_An_set_thr_temperature(float val)
{
	aqu_info.anm_thr_temperature = val;
}

void AQU_An_set_rv_open_time(unsigned short val)
{
	aqu_info.anm_thr_time_rv_open = val;
}

void AQU_An_set_tv_open_time(unsigned short val)
{
	aqu_info.anm_thr_time_tv_open = val;
}

//Anomalyˆ—ŽÀs
static void AQU_An_execute_(void)
{

	static unsigned int c;
	if (c >= (0.2 * OBCT_CYCLES_PER_SEC))
	{
		c = 0;
		if (aqu_info.is_enabled)
		{
			//ˆ³—Í·•ªŒvŽZ
			aqu_info.tlm.AQU_DIF_PRESSURE_TPS_01 = aqu_info.tlm.AQU_PRESSURE_TPS_01 - aqu_info.tlm.AQU_OLD_PRESSURE_TPS_01;
			aqu_info.tlm.AQU_DIF_PRESSURE_TPS_02 = aqu_info.tlm.AQU_PRESSURE_TPS_02 - aqu_info.tlm.AQU_OLD_PRESSURE_TPS_02;

			//ANM_01///////////////////////////////////////////////////////////////////////
			AQU_Anm_detect_func((unsigned short)aqu_info.tar_tps, AQU_TAR_BIT_TPS_01, aqu_info.tlm.AQU_DIF_PRESSURE_TPS_01,
				aqu_info.anm_thr_diff_ps, AQU_ANM_01);

			AQU_Anm_detect_func((unsigned short)aqu_info.tar_tps, AQU_TAR_BIT_TPS_02, aqu_info.tlm.AQU_DIF_PRESSURE_TPS_02,
				aqu_info.anm_thr_diff_ps, AQU_ANM_01);
				
			//ANM_02,03/////////////////////////////////////////////////////////////////////

			if (aqu_info.tlm.AQU_VALVE_STATE_RV1 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV1++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV1 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rv1, AQU_TAR_BIT_RV_01, aqu_info.anm_thr_time_rv_open,
				AQU_ANM_02, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV1);


			if (aqu_info.tlm.AQU_VALVE_STATE_RV2 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV2++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV2 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rv2, AQU_TAR_BIT_RV_02, aqu_info.anm_thr_time_rv_open,
				AQU_ANM_03, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV2);


			if (aqu_info.tlm.AQU_VALVE_STATE_RV3 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV3++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV3 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rv1, AQU_TAR_BIT_RV_03, aqu_info.anm_thr_time_rv_open,
				AQU_ANM_02, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV3);

			if (aqu_info.tlm.AQU_VALVE_STATE_RV4 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV4++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV4 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rv2, AQU_TAR_BIT_RV_02, aqu_info.anm_thr_time_rv_open,
				AQU_ANM_03, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RV4);


			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_TTS,
				aqu_info.anm_thr_temperature, AQU_ANM_04);
			

			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_WTS,
				aqu_info.anm_thr_temperature, AQU_ANM_05);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_VTS1,
				aqu_info.anm_thr_temperature, AQU_ANM_06);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_VTS2,
				aqu_info.anm_thr_temperature, AQU_ANM_07);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_VTS3,
				aqu_info.anm_thr_temperature, AQU_ANM_08);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_VTS4,
				aqu_info.anm_thr_temperature, AQU_ANM_09);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_VTS5,
				aqu_info.anm_thr_temperature, AQU_ANM_10);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_VTS6,
				aqu_info.anm_thr_temperature, AQU_ANM_11);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_DTS,
				aqu_info.anm_thr_temperature, AQU_ANM_12);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_RTS1,
				aqu_info.anm_thr_temperature, AQU_ANM_13);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_RTS2,
				aqu_info.anm_thr_temperature, AQU_ANM_14);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_RTS3,
				aqu_info.anm_thr_temperature, AQU_ANM_15);
			AQU_Anm_detect_func(aqu_info.tar_tmp_sens, AQU_TAR_BIT_TEMP, aqu_info.tlm.AQU_TEMPERATURE_RTS4,
				aqu_info.anm_thr_temperature, AQU_ANM_16);


			//ANM_17, 20/////////////////////////////////////////////////////////////////////
			if (aqu_info.tlm.AQU_VALVE_STATE_RTV1 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV1++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV1 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rtv_a, AQU_TAR_BIT_RTV_01, aqu_info.anm_thr_time_tv_open,
				AQU_ANM_17, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV1);

			if (aqu_info.tlm.AQU_VALVE_STATE_RTV2 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV2++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV2 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rtv_b, AQU_TAR_BIT_RTV_02, aqu_info.anm_thr_time_tv_open,
				AQU_ANM_18, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV2);

			if (aqu_info.tlm.AQU_VALVE_STATE_RTV3 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV3++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV3 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rtv_c, AQU_TAR_BIT_RTV_03, aqu_info.anm_thr_time_tv_open,
				AQU_ANM_18, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV3);

			if (aqu_info.tlm.AQU_VALVE_STATE_RTV4 == 1) { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV4++; }
			else { aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV4 = 0; }
			AQU_Anm_detect_open_time_func(aqu_info.tar_rtv_b, AQU_TAR_BIT_RTV_04, aqu_info.anm_thr_time_tv_open,
				AQU_ANM_18, aqu_info.tlm.AQU_VALVE_OPEN_TIME_RTV4);

			///OLD’lŠi”[
			aqu_info.tlm.AQU_OLD_PRESSURE_TPS_01 = aqu_info.tlm.AQU_PRESSURE_TPS_01;
			aqu_info.tlm.AQU_OLD_PRESSURE_TPS_02 = aqu_info.tlm.AQU_PRESSURE_TPS_02;
		}

	}
	c++;

	
}


//AQUƒeƒŒƒƒgƒŠŽæ“¾
void AQU_TlmExtract(CTCP *packet)
{
	unsigned char	c_tmp = 0;
	unsigned int	ui_tmp = 0;
	unsigned short	us_tmp = 0;
	unsigned int i;
	unsigned int ofs = 0;

	aqu_info.tar_tps   = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	aqu_info.tar_rv1   = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	aqu_info.tar_rv2   = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	aqu_info.tar_rtv_a = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	aqu_info.tar_rtv_b = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	aqu_info.tar_rtv_c = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	aqu_info.tar_rtv_d = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;

	ui_tmp = 0;
	for (i = 0; i<4; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_ZEUS_TIME = ui_tmp; ofs += 4;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_RTS2 = AQU_TEMP_SCALE * (float)ui_tmp + AQU_TEMP_OFFSET; ofs += 2;
	
	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_VTS3 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_VTS5 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_RTS1 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_RTS4 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_VTS1 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_VTS6 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_WTS = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_VTS2 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_RTS3 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_TTS = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_VTS4 = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_TEMPERATURE_DTS = AQU_TEMP_SCALE * ui_tmp + AQU_TEMP_OFFSET; ofs += 2;

	ofs += 2;
	ofs += 2;
	ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_PRESSURE_VPS_01 = AQU_VPS_SCALE * ui_tmp + AQU_VPS_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_PRESSURE_TPS_01 = AQU_TPS_SCALE * ui_tmp + AQU_TPS_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_PRESSURE_TPS_02 = AQU_TPS_SCALE * ui_tmp + AQU_TPS_OFFSET; ofs += 2;

	//
	ui_tmp = 0;
	for (i = 0; i<2; ++i)
	{
		ui_tmp <<= 8;
		ui_tmp += packet->data[TLM_POSITION_MAIN + i + ofs];
	}
	aqu_info.tlm.AQU_PRESSURE_VPS_02 = AQU_VPS_SCALE * ui_tmp + AQU_VPS_OFFSET; ofs += 2;

	ofs += 2;

	//
	aqu_info.tlm.AQU_VALVE_STATE_RV1 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;
	
	aqu_info.tlm.AQU_VALVE_STATE_RV2 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_RV3 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_RV4 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_VDV = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_DTV1 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_DTV2 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_DTV3 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_DTV4 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_RTV1 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;
	/*
	aqu_info.tlm.AQU_VALVE_STATE_RTV1 = 1;
	Serial.print("aqu_info.tlm.AQU_VALVE_STATE_RTV1: ");
	Serial.println(aqu_info.tlm.AQU_VALVE_STATE_RTV1);
	*/

	aqu_info.tlm.AQU_VALVE_STATE_RTV2 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;

	aqu_info.tlm.AQU_VALVE_STATE_RTV3 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;
	
	aqu_info.tlm.AQU_VALVE_STATE_RTV4 = packet->data[TLM_POSITION_MAIN + ofs];	ofs++;
	ofs += 2;
}

void AQU_Anm_detect_func(unsigned short tar, unsigned short tar_bit, float val, float thr, unsigned char anm_sts)
{
	if ((tar & tar_bit) > 0)
	{
		if (val > thr)
		{
			aqu_info.sts = anm_sts;
			AL_add_anomaly(AL_AQU, anm_sts);
		}
	}
}

void AQU_Anm_detect_open_time_func(unsigned short tar, unsigned short tar_bit, unsigned short thr, unsigned char anm_sts, unsigned short times)
{
	if ((tar & tar_bit) > 0)
	{
		if (times > thr)
		{
			aqu_info.sts = anm_sts;
			AL_add_anomaly(AL_AQU, anm_sts);
		}
	}
}

void AQU_Anm_detect_time_func(unsigned short tar, unsigned short tar_bit, unsigned char sts, unsigned short thr, 
	unsigned char anm_sts, unsigned short *times)
{
	unsigned char flg = (tar & tar_bit);
	if (flg > 0)
	{
		if (sts)
		{
			*times += 1;
			if (*times > thr)
			{
				aqu_info.sts = anm_sts;
				AL_add_anomaly(AL_AQU, anm_sts);
				*times = 0;
			}
		}
		else {
			*times = 0;
		}
	}
}