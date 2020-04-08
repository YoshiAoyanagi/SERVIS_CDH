// 
// 
// 

#include "cmd_action.h"


int Cmd_SET_OBC_TIME(const CTCP* packet)
{
	int i;
	unsigned int cycle;

	for (i = 0; i<4; ++i)
	{
		cycle <<= 8;
		cycle += packet->data[CMD_POSITION_PARAM + i];
	}
	TMGR_set_master_cycle((cycle_t)cycle);
	TMGR_set_initial_utime(0);
	TDSP_resync_internal_counter();

	return CMD_ERROR_NONE;
}

int Cmd_SYNC_TIME(const CTCP* packet)
{
	int i;
	unsigned int gs_utime;

	for (i = 0; i<4; ++i)
	{
		gs_utime <<= 8;
		gs_utime += packet->data[CMD_POSITION_PARAM + i];
	}
	RTC_UTIME_SET(gs_utime);
	if (TMGR_sync_OBC_time(gs_utime) == -1)
		return CMD_ERROR_ILLEGAL_PARAMETER;
	return CMD_ERROR_NONE;
}

int Cmd_SYNC_OBC_TIME(const CTCP* packet)
{
	int i;
	unsigned int gs_utime;

	for (i = 0; i<4; ++i)
	{
		gs_utime <<= 8;
		gs_utime += packet->data[CMD_POSITION_PARAM + i];
	}
	if (TMGR_sync_OBC_time(gs_utime) == -1)
		return CMD_ERROR_ILLEGAL_PARAMETER;
	return CMD_ERROR_NONE;
}

int Cmd_SYNC_RTC_TIME(const CTCP* packet)
{
	int i;
	unsigned int gs_utime;

	for (i = 0; i<4; ++i)
	{
		gs_utime <<= 8;
		gs_utime += packet->data[CMD_POSITION_PARAM + i];
	}

	RTC_UTIME_SET(gs_utime);
	return CMD_ERROR_NONE;
}

int Cmd_SYNC_GPS_TIME(const CTCP* packet)
{
	return CMD_ERROR_NONE;
}

int Cmd_FRAM_SYNC_RTC_ENABLE(const CTCP* packet)
{
	unsigned char ena = packet->data[CMD_POSITION_PARAM];
	FRAM_Write_rtc_sync_ena((ENABLE_STS)ena);
	return CMD_ERROR_NONE;
}

int Cmd_SOFTWARE_RESET(const CTCP* packet)
{
	return CMD_ERROR_NONE;
}
int Cmd_FRAM_INITIALIZE(const CTCP* packet)
{
	FRAM_All_Initialize();
	return CMD_ERROR_NONE;
}

int Cmd_FRAM_CMD_INITIALIZE(const CTCP* packet)
{
	FRAM_TLC_All_Initialize();
	return CMD_ERROR_NONE;
}

int Cmd_FRAM_TLC_CLEAR_ALL(const CTCP* packet)
{
	FRAM_TLC_Clear_All();
	return CMD_ERROR_NONE;
}
int Cmd_FRAM_TLC_CLEAR_AT_TIME(const CTCP* packet)
{
	int i;
	unsigned int utime;

	for (i = 0; i<4; ++i)
	{
		utime <<= 8;
		utime += packet->data[CMD_POSITION_PARAM + i];
	}
	if(FRAM_TLC_Clear_At_Time(utime) == Error)
		return CMD_ERROR_ILLEGAL_PARAMETER;

	return CMD_ERROR_NONE;
}
int Cmd_FRAM_TLC_CLEAR_AT_NUMBER(const CTCP* packet)
{
	unsigned char p = packet->data[CMD_POSITION_PARAM];
	FRAM_TLC_Clear_At_Address((unsigned int)p);

	return CMD_ERROR_NONE;
}


int Cmd_FRAM_DEPLOY_BLOCK(const CTCP* packet)
{
	unsigned char bc_id = packet->data[CMD_POSITION_PARAM];
	FRAM_SBC_Deploy_Block(bc_id);
	return CMD_ERROR_NONE;
}
int Cmd_FRAM_REMOVE_BLOCK(const CTCP* packet)
{
	unsigned char bc_id = packet->data[CMD_POSITION_PARAM];
	FRAM_SBC_Remove_Block(bc_id);
	return CMD_ERROR_NONE;
}
int Cmd_FRAM_REMOVE_BLOCK_AT_POSITION(const CTCP* packet)
{
	unsigned char bc_id = packet->data[CMD_POSITION_PARAM];
	unsigned char pos = packet->data[CMD_POSITION_PARAM + 1];
	FRAM_SBC_Remove_Block_At_PS(bc_id, pos);
	return CMD_ERROR_NONE;
}
int Cmd_FRAM_REMOVE_ALL_BLOCK(const CTCP* packet)
{
	FRAM_SBC_Remove_ALL_Block();
	return CMD_ERROR_NONE;
}
int Cmd_FRAM_SET_BLOCK_POSITION(const CTCP* packet)
{
	unsigned char number = packet->data[CMD_POSITION_PARAM];
	unsigned char pos = packet->data[CMD_POSITION_PARAM + 1];

	if (FRAM_SBC_Set_Stored_BC_Number(number, pos) != Success)
		return CMD_ERROR_ILLEGAL_PARAMETER;

	return CMD_ERROR_NONE;
}

int Cmd_FRAM_ACTIVATE_BLOCK(const CTCP* packet)
{
	unsigned char bc_id = packet->data[CMD_POSITION_PARAM];
	unsigned char activate = packet->data[CMD_POSITION_PARAM + 1];

	FRAM_SBC_ACTIVATE(bc_id, activate);
	return CMD_ERROR_NONE;
}

int Cmd_FRAM_REQUEST_BC_INFO(const CTCP* packet)
{
	unsigned char p = packet->data[CMD_POSITION_PARAM];
	FRAM_SBC_Read_BlockInfo(p);
	
	OBC_TlmRegister(TLM_SBC);
	return CMD_ERROR_NONE;
}

int Cmd_FRAM_REQUEST_BC_PARAM(const CTCP* packet)
{
	unsigned char id = packet->data[CMD_POSITION_PARAM];
	unsigned char pos = packet->data[CMD_POSITION_PARAM + 1];

	FRAM_SBC_ExtractData(id, pos);

	OBC_TlmRegister(TLM_SBC_PARAM);
	return CMD_ERROR_NONE;
}
