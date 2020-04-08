// 
// 
// 

#include "cmd_action_cmdtlm.h"
#include "cmd_analyze_obc.h"
#include "Stored_TlmGenerator.h"
#include "STRX_update.h"

int Cmd_TlmRequest(const CTCP* packet)
{
	unsigned char id = packet->data[CMD_POSITION_PARAM];
	int i_ret = OBC_TlmRegister(id);
	return i_ret;
}

int Cmd_SET_TLM_INTERVAL(const CTCP* packet)
{
	unsigned int interval_sec;
	
	interval_sec = (packet->data[CMD_POSITION_PARAM] << 8);
	interval_sec += packet->data[CMD_POSITION_PARAM + 1];
	OBC_Tlm_Set_tlm_interval(interval_sec);
	return CMD_ERROR_NONE;
}

int Cmd_TLM_SET_MODE(const CTCP* packet)
{
	return CMD_ERROR_NONE;
}

int Cmd_MUX_UART_SET(const CTCP* packet)
{
	unsigned char port = packet->data[CMD_POSITION_PARAM];
	Serial_MUX_change((SERIAL_MUX)port);

	return CMD_ERROR_NONE;
}




///Cmd
int Cmd_OBC_DR_REPLAY_PARAM_SET(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned int start_sector;
	unsigned int start_packet_address;

	for (i = 0; i < 2; i++)
	{
		start_sector <<= 8;
		start_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		start_packet_address <<= 8;
		start_packet_address += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;
	hk_dr.replay_pointer = DR_Sector_Init(start_sector, start_packet_address);

	return CMD_ERROR_NONE;
}

int Cmd_OBC_DR_REPLAY(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned char _enable;
	unsigned int _num_packet;
	unsigned int _interval_time;

	_enable = packet->data[CMD_POSITION_PARAM + ofs];
	ofs += 1;

	for (i = 0; i < 2; i++)
	{
		_num_packet <<= 8;
		_num_packet += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_interval_time <<= 8;
		_interval_time += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	hk_dr.replay_interval_time = _interval_time;
	hk_dr.replay_num_packet = _num_packet;
	hk_dr.replay_enabled = _enable;

	return CMD_ERROR_NONE;
}

//Add 2019/11/13
int Cmd_OBC_DR_DIRECT_REPLAY(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned char _enable;
	unsigned int _num_packet;
	unsigned int _interval_time;
	unsigned int start_sector;
	unsigned int start_packet_address;

	for (i = 0; i < 2; i++)
	{
		start_sector <<= 8;
		start_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		start_packet_address <<= 8;
		start_packet_address += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_num_packet <<= 8;
		_num_packet += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_interval_time <<= 8;
		_interval_time += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	hk_dr.replay_pointer = DR_Sector_Init(start_sector, start_packet_address);
	hk_dr.replay_interval_time = _interval_time;
	hk_dr.replay_num_packet = _num_packet;
	hk_dr.replay_enabled = 1;

	return CMD_ERROR_NONE;
}


int Cmd_OBC_DR_ERASE(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned int _start_sector;
	unsigned int _end_sector;

	for (i = 0; i < 2; i++)
	{
		_start_sector <<= 8;
		_start_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_end_sector <<= 8;
		_end_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	st_tlm_erase(_start_sector, _end_sector);

	return CMD_ERROR_NONE;
}

int Cmd_OBC_DR_Replay_Stop(const CTCP* packet)
{
	hk_dr.replay_enabled = 0;
	return CMD_ERROR_NONE;
}

int Cmd_OBC_DR_RECORD(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned char _enable;
	unsigned int _interval = 0;

	_enable = packet->data[CMD_POSITION_PARAM + ofs];
	ofs++;

	for (i = 0; i < 2; i++)
	{
		_interval <<= 8;
		_interval += packet->data[CMD_POSITION_PARAM + ofs + i];
	}

	if (_enable > 1)
		return CMD_ERROR_ILLEGAL_PARAMETER;

	if (_interval == 0)
		_enable = 0;

	hk_dr.enabled = _enable;
	st_tlm_set_interval(_interval);
	return CMD_ERROR_NONE;
}


//mission recorder
int Cmd_MSN_DR_REPLAY_PARAM_SET(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned int start_sector;
	unsigned int start_packet_address;

	for (i = 0; i < 2; i++)
	{
		start_sector <<= 8;
		start_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		start_packet_address <<= 8;
		start_packet_address += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;
	msn_dr.replay_pointer = DR_Sector_Init(start_sector, start_packet_address);

	return CMD_ERROR_NONE;
}

int Cmd_MSN_DR_REPLAY(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned char _enable;
	unsigned int _num_packet;
	unsigned int _interval_time;

	_enable = packet->data[CMD_POSITION_PARAM + ofs];
	ofs += 1;

	for (i = 0; i < 2; i++)
	{
		_num_packet <<= 8;
		_num_packet += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_interval_time <<= 8;
		_interval_time += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	msn_dr.replay_interval_time = _interval_time;
	msn_dr.replay_num_packet = _num_packet;
	msn_dr.replay_enabled = _enable;

	return CMD_ERROR_NONE;
}

int Cmd_MSN_DR_ERASE(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned short _start_sector;
	unsigned short _end_sector;

	for (i = 0; i < 2; i++)
	{
		_start_sector <<= 8;
		_start_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_end_sector <<= 8;
		_end_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	MSN_DR_Set_EraseMode(_start_sector, _end_sector);//
													 //MSN_DR_Erase(_start_sector, _end_sector); //

	return CMD_ERROR_NONE;
}

int Cmd_MSN_DR_Replay_Stop(const CTCP* packet)
{
	msn_dr.replay_enabled = 0;
	return CMD_ERROR_NONE;
}

int Cmd_MSN_DR_RECORD_ENABLE(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned char _enable;

	_enable = packet->data[CMD_POSITION_PARAM + ofs];
	ofs++;

	if (_enable > 1)
		return CMD_ERROR_ILLEGAL_PARAMETER;

	msn_dr.enabled = _enable;
	return CMD_ERROR_NONE;
}

int Cmd_OBC_DR_INITIALIZE(const CTCP* packet)
{
	OBC_DR_Initialize();
	return CMD_ERROR_NONE;
}

int Cmd_MSN_DR_INITIALIZE(const CTCP* packet)
{
	MSN_DR_Initialize();
	return CMD_ERROR_NONE;
}

//Add 2019/11/13
int Cmd_MSN_DR_DIRECT_REPLAY(const CTCP* packet)
{
	int i;
	int ofs = 0;

	unsigned char _enable;
	unsigned int _num_packet;
	unsigned int _interval_time;
	unsigned int start_sector;
	unsigned int start_packet_address;

	for (i = 0; i < 2; i++)
	{
		start_sector <<= 8;
		start_sector += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		start_packet_address <<= 8;
		start_packet_address += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_num_packet <<= 8;
		_num_packet += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	for (i = 0; i < 2; i++)
	{
		_interval_time <<= 8;
		_interval_time += packet->data[CMD_POSITION_PARAM + i + ofs];
	}
	ofs += 2;

	msn_dr.replay_pointer = DR_Sector_Init(start_sector, start_packet_address);
	msn_dr.replay_interval_time = _interval_time;
	msn_dr.replay_num_packet = _num_packet;
	msn_dr.replay_enabled = 1;

	return CMD_ERROR_NONE;
}


//STRX
int Cmd_STRX_SET_BR(const CTCP* packet)
{
	unsigned char param = packet->data[CMD_POSITION_PARAM ];

	if(param == 0)
		STRX_Cmd_BR4KBPS();
	else if (param == 1)
		STRX_Cmd_BR32KBPS();
	else if(param == 2)
		STRX_Cmd_BR64KBPS();
	else
		return CMD_ERROR_ILLEGAL_PARAMETER;

	return CMD_ERROR_NONE;
}
int Cmd_STRX_SET_TX_PWR(const CTCP* packet)
{
	unsigned char param = packet->data[CMD_POSITION_PARAM];

	if(param > 0x0F)
		return CMD_ERROR_ILLEGAL_PARAMETER;

	STRX_Cmd_PWR(param);

	return CMD_ERROR_NONE;
}
int Cmd_STRX_TX_ON(const CTCP* packet)
{
	unsigned char param = packet->data[CMD_POSITION_PARAM];

	if (param == 0)
		STRX_IF_Cmd_TX_OFF();
	else if (param == 1)
		STRX_IF_Cmd_TX_ON();
	else
		return CMD_ERROR_ILLEGAL_PARAMETER;

	return CMD_ERROR_NONE;
}

int Cmd_SET_STX_ONOFF_DURATION(const CTCP* packet)
{
	int i;
	int ofs = 0;
	unsigned int _on_dur = 0;
	unsigned int _off_dur = 0;

	for (i = 0; i < 2; i++)
	{
		_on_dur <<= 8;
		_on_dur += packet->data[CMD_POSITION_PARAM + ofs + i];
	}
	ofs+=2;
	for (i = 0; i < 2; i++)
	{
		_off_dur <<= 8;
		_off_dur += packet->data[CMD_POSITION_PARAM + ofs + i]; 
	}
	ofs+=2;

	STX_set_transmit_duration(_on_dur, _off_dur);

	return CMD_ERROR_NONE;
}
int Cmd_SET_STX_BURST_DURATION(const CTCP* packet)
{
	int i;
	int ofs = 0;
	unsigned int _on_dur = 0;
	
	for (i = 0; i < 2; i++)
	{
		_on_dur <<= 8;
		_on_dur += packet->data[CMD_POSITION_PARAM + ofs + i];
	}
	if (_on_dur > (30 * 60))	//30•ªˆÈã‚Ì˜A‘±ON‚Í”F‚ß‚È‚¢
		return CMD_ERROR_ILLEGAL_PARAMETER;
	STX_set_burst_duration(_on_dur);

	return CMD_ERROR_NONE;
}

int Cmd_STRX_OTC_ENABLE(const CTCP* packet)
{
	unsigned char tmp = packet->data[CMD_POSITION_PARAM];
	if(tmp > 1)
		return CMD_ERROR_ILLEGAL_PARAMETER;

	STRX_set_OTC_Enable(tmp);
	return CMD_ERROR_NONE;
}
int Cmd_STRX_OTC_SET_THRESHOLD(const CTCP* packet)
{
	unsigned char tmp = packet->data[CMD_POSITION_PARAM];
	if (tmp < STRX_OTC_LOW_TEMPERATURE)
		return CMD_ERROR_ILLEGAL_PARAMETER;

	STRX_set_OTC_Threshold(tmp);
	return CMD_ERROR_NONE;
}

int Cmd_OBC_BIN_DATA_UPLOAD(const CTCP* packet)
{
	static unsigned char count;
	CTCP tlm_packet = CCP_initialize(packet->data, packet->length);

	CCP_set_tlm_id(&tlm_packet, 0xA0);
	CCP_set_tlm_count(&tlm_packet, count);
	if (count == 255)
		count = 0;
	else
		count++;

	OBC_DR_PacketWrite(&tlm_packet);
	return CMD_ERROR_NONE;
}