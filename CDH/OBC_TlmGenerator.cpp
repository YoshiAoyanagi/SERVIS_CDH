// 
// 
// 

#include "OBC_TlmGenerator.h"
#include <string.h> // for memcmp

#include "utility.h"
#include "PacketHandler.h"
#include "cmd_analyze_obc.h"
#include "tlm_frame.h"
#include "BlockCommandDefinisions.h"

void tlm_initialize_(void);
void OBC_tlm_generator_(void);
unsigned int tlm_send_interval;

AppInfo OBC_tlm_generator(void)
{
	return create_app_info("TLM_GEN", 2, tlm_initialize_, OBC_tlm_generator_);
}

void tlm_initialize_(void)
{
	tlm_send_interval = TLM_INITAL_SEND_INTERVAL;
}

void OBC_tlm_generator_(void)
{
	int ofs;

	static unsigned int c;
	if (c >= (tlm_send_interval * OBCT_CYCLES_PER_SEC))
	{
		c = 0;
		ofs = MakeTlmPacket(TLM_ID_01, TLM_REAL);
		if (ofs > 0)
		{
			CTCP tlm_packet = CCP_initialize(TlmTransferFrame.Packet.packet_data, (size_t)(TLM_BYTE_COUNT + ofs));
			PH_analyze_packet(&tlm_packet);
		}
	}
	c++;
}

int OBC_TlmSend(unsigned char id)
{
	int ofs = 0;

	// �͈͊O��TLM ID�����O
	if (id > 0xff) { return CMD_ERROR_ILLEGAL_PARAMETER; }
	// ���݂��Ȃ�TLM ID�����O
	if (tlm_packet_ptr[id] == NULL) { return CMD_ERROR_ILLEGAL_CONTEXT; }

	// TLM�p�P�b�g����+���o
	ofs = TlmPacket_combining(id, tlm_packet_ptr[id], TLM_REAL);
	if (ofs > 0)
	{
		TlmSend(ofs);
	}
	return CMD_ERROR_NONE;
}

int OBC_TlmRegister(unsigned char id)
{
	// �͈͊O��TLM ID�����O
	if (id > 0xff) { return CMD_ERROR_ILLEGAL_PARAMETER; }
	// ���݂��Ȃ�TLM ID�����O
	if (tlm_packet_ptr[id] == NULL) { return CMD_ERROR_ILLEGAL_CONTEXT; }

	int ofs = MakeTlmPacket(id, TLM_REAL);
	if (ofs > 0)
	{
		CTCP tlm_packet = CCP_initialize(TlmTransferFrame.Packet.packet_data, (size_t)(TLM_BYTE_COUNT + ofs));
		PH_analyze_packet(&tlm_packet);
	}
	return CMD_ERROR_NONE;
}
ack OBC_Tlm_Set_tlm_interval(unsigned int interval_sec)
{
	tlm_send_interval = interval_sec;
	return Success;
}