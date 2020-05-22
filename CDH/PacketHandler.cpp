#include "PacketHandler.h"
#include "FRAM_TL_CmdDispatcher.h"
#include "DataRecorderManager.h"
#include "AQU_Anomaly_handler.h"

PL_Info rt_tlm_list;
PL_Info tl_cmd_list;
PL_Info rt_cmd_list;
PL_Info to_msn_cmd_list;
PL_Info to_adcs_cmd_list;

int GsCmdSts;
PH_ACK GsCmdErrorSts;
int GsCmdCode;
int GsCmdToId;
int GsCmdCount;
int GsTlmDsc;
int CdhCmdCount;
int CdhCmdCode;

static PL_Node rt_cmd_stock_[RT_CMD_LIST_MAX];
static PL_Node rt_tlm_stock_[RT_TLM_LIST_MAX];
static PL_Node tl_cmd_stock_[TL_CMD_LIST_MAX];
static PL_Node to_adcs_cmd_stock_[TO_ADCS_CMD_LIST_MAX];
static PL_Node to_msn_cmd_stock_[TO_MSN_CMD_LIST_MAX];

static PH_ACK analyze_block_cmd_(const CTCP* packet);
static PH_ACK analyze_single_cmd_(CTCP* packet);
static PH_ACK analyze_cmd_(CTCP* packet);

static PH_ACK add_rt_cmd_(const CTCP* packet);
static PH_ACK add_tl_cmd_(const CTCP* packet, size_t now);
static PH_ACK add_adcs_cmd_(const CTCP* packet);
static PH_ACK add_msn_cmd_(const CTCP* packet);
static PH_ACK add_fram_tlc(const CTCP* packet, utime_t now);

PE_Info PE_initialize(void)
{
	PE_Info pei;
	pei.packet = CCP_initialize(NULL, 0);
	pei.rec_status = CMD_REC_STATUS_0;
	pei.rec_count = 0;

	return pei;
}

void PH_init(void)
{
	rt_tlm_list = PL_initialize(rt_tlm_stock_, RT_TLM_LIST_MAX);
	rt_cmd_list = PL_initialize(rt_cmd_stock_, RT_CMD_LIST_MAX);
	tl_cmd_list = PL_initialize(tl_cmd_stock_, TL_CMD_LIST_MAX);
	to_msn_cmd_list = PL_initialize(to_msn_cmd_stock_, TO_MSN_CMD_LIST_MAX);
	to_adcs_cmd_list = PL_initialize(to_adcs_cmd_stock_, TO_ADCS_CMD_LIST_MAX);

	// �n��R�}���h�����X�e�[�^�X�̏�����
	GsCmdSts = CMD_STATUS_NONE;
	GsCmdErrorSts = PH_UNKNOWN;
	GsCmdCode = 0;
	GsCmdToId = 0;
	GsTlmDsc = 0;
	CdhCmdCount = 0;
	CdhCmdCode = 0;
	
	return;
}
static int PH_check_packet_error(CTCP* packet)
{
	unsigned int	crc_calc;
	unsigned int	crc_cmd;

	unsigned char id = packet->data[TLM_POSITION_TLM_MODE];

	//	ETX Check
	if (packet->data[packet->length - 1] != CMD_ETX2) {
		return(-1);
	}
	if (packet->data[packet->length - 2] != CMD_ETX1) {
		return(-1);
	}

	//	CRC Check
	crc_calc = crc(&(packet->data[CMD_POSITION_TLMCMD]), (packet->length - 2 - 1 - 2 - 2));	// -STX*2 - LENGTH   -ETX*2 - CRC*2 
	crc_calc &= 0xffff;
	crc_cmd = packet->data[packet->length - 4];
	crc_cmd <<= 8;
	crc_cmd |= packet->data[packet->length - 3];

	if (crc_calc != crc_cmd)
		return(-2);
	
	return(0);
}

PH_ACK PH_extract_packet(unsigned char *rec_buffer, int rec_byte, PE_Info* pei)
{
	PH_ACK ack = PH_NOW_PROCESSING;//PH_UNKNOWN;

	while (rec_byte > 0)
	{
		if (pei->rec_status == CMD_REC_STATUS_0) // STX
		{
			if (*rec_buffer == CMD_STX1)
			{
				pei->packet.length = 0;
				pei->packet.data[pei->packet.length] = *rec_buffer;
				pei->packet.length++;
				pei->rec_status = CMD_REC_STATUS_1;
			}
		}
		else if (pei->rec_status == CMD_REC_STATUS_1) // STX
		{
			if (*rec_buffer == CMD_STX2)
			{
				pei->packet.data[pei->packet.length] = *rec_buffer;
				pei->packet.length++;
				pei->rec_status = CMD_REC_STATUS_2;
			}
		}
		else if (pei->rec_status == CMD_REC_STATUS_2) // Length
		{
			pei->packet.data[pei->packet.length] = *rec_buffer;
			pei->packet.length++;
			pei->rec_count = *rec_buffer;
			pei->rec_count += 4; // IncludeCRC and ETX

			pei->rec_status = CMD_REC_STATUS_3;
		}
		else if (pei->rec_status == CMD_REC_STATUS_3) // CMD_DATA
		{
			pei->packet.data[pei->packet.length] = *rec_buffer;
			pei->packet.length++;
			pei->rec_count--;
			if (pei->rec_count <= 0)
			{
				int ret = 0;
				ret = PH_check_packet_error(&(pei->packet));
				if (ret == 0)
				{
					ack = PH_analyze_packet(&(pei->packet));
				}
				else if (ret == -1)
				{
					ack = PH_INVALID_LENGTH;
					Serial.println("length error");
				}
				else if (ret == -2)
				{
					ack = PH_INVALID_CRC;
					Serial.println("crc error");
				}
				pei->rec_status = CMD_REC_STATUS_0;
			}
		}
		rec_buffer++;
		rec_byte--;
	}
	return ack;
}

PH_ACK PH_analyze_packet(CTCP* packet)
{
	switch (CCP_get_tlm_cmd_discrimination(packet))
	{
	case TLMCMD_CMD:
		//UTIME��cycle�ɕϊ�
		return analyze_cmd_(packet);
	case TLMCMD_SC_CMD:
		CCP_set_tlm_cmd_discrimination(packet, TLMCMD_CMD);
		return analyze_cmd_(packet);
	case TLMCMD_TLM:
		return add_rt_tlm(packet);
	case TLMCMD_DR:
		return add_rt_tlm(packet);
	default:
		return PH_INVALID_DISCRIMINATOR;
	}
	return PH_UNKNOWN;
}

static PH_ACK analyze_cmd_(CTCP* packet)
{
	switch (CCP_get_cmd_type_id(packet))
	{
	case CCP_SINGLE:
		return analyze_single_cmd_(packet);
	case CCP_BLOCK:
		if (CCP_get_cmd_from_id(packet) == FROM_TO_GS)
		{
			CCP_set_utime2cycle(packet);
		}
		return analyze_block_cmd_(packet);
	case CCP_STORED_BLOCK:
		FRAM_SBC_Write_packet(packet);
		return PH_BC_REGISTERED;
	default:
		return PH_UNKNOWN;
	}
}

static PH_ACK analyze_block_cmd_(const CTCP* packet)
{
	switch (BCT_register_cmd(packet))
	{
	case BCT_SUCCESS:
		++(rt_cmd_list.pl_public.executed_nodes);	//BC, TLC�ł�cmd_count���㏸
		return PH_BC_REGISTERED;
	case BCT_INVALID_BLOCK_NO:
		return PH_BC_INVALID_BLOCK_NO;
	case BCT_INVALID_CMD_NO:
		return PH_BC_INVALID_CMD_NO;
	case BCT_ISORATED_CMD:
		return PH_BC_ISORATED_CMD;
	default:
		return PH_UNKNOWN;
	}
}

PH_ACK PH_Cmd_Router(const CTCP* packet)
{
	switch (CCP_get_cmd_to_id(packet))
	{
	case FROM_TO_COMM:
		return PH_despatch_command(packet);
	case FROM_TO_ADCS:
		return add_adcs_cmd_(packet);
	case FROM_TO_SF:
		return add_msn_cmd_(packet);
	case FROM_TO_SUBCAM:
		return add_msn_cmd_(packet);
	case FROM_TO_OVCO:
		return add_msn_cmd_(packet);
	case FROM_TO_AQUA:
		return add_msn_cmd_(packet);
	default:return PH_FORWARDED;
	}
	return PH_UNKNOWN;
}

static PH_ACK analyze_single_cmd_(CTCP* packet)
{
	switch (CCP_get_cmd_exe_type(packet))
	{
	case CCP_REALTIME:
		return PH_Cmd_Router(packet);
	case CCP_TIMELINE:
		if (CCP_get_cmd_from_id(packet) == FROM_TO_GS)
		{
			CCP_set_utime2cycle(packet);
		}
		// �R�}���h���s������OBC�ɏ㏑��
		CCP_set_cmd_from_id(packet, FROM_TO_COMM);
		++(rt_cmd_list.pl_public.executed_nodes);		//BC, TLC�ł�cmd_count���㏸
		return add_tl_cmd_(packet, (size_t)master_clock->master);
	case CCP_FRAM_TLC:
		// �R�}���h���s������OBC�ɏ㏑��
		CCP_set_cmd_from_id(packet, FROM_TO_COMM);
		CCP_set_cmd_exe_type(packet, CCP_REALTIME);
		++(rt_cmd_list.pl_public.executed_nodes);		//BC, TLC�ł�cmd_count���㏸
		return add_fram_tlc(packet, master_clock->utime);
	default:
		return PH_UNKNOWN;
	}
}
PH_ACK PH_despatch_command(const CTCP* packet)
{
	// CMD�ȊO�̃p�P�b�g��������ُ픻��
	int tcd = CCP_get_tlm_cmd_discrimination(packet);
	if ((tcd != TLMCMD_CMD))
	{
		return PH_INVALID_DISCRIMINATOR;
	}

	// Single�R�}���h�̏ꍇ
	if (CCP_get_cmd_type_id(packet) == CCP_SINGLE)
	{
		PH_ACK ack;
		if (CCP_get_cmd_to_id(packet) == FROM_TO_COMM)
		{
			ack = (PH_ACK)cmdExec(packet);
		}
		else
		{
			ack = PH_Cmd_Router(packet);
		}
		return ack;
	}
	// Single�ȊO�̏ꍇ�ُ͈픻��
	else { return PH_INVALID_CMD_TYPE; }
}

PH_ACK PH_send_realtime_telemetry(CTCP* packet)
{
	int i;
	static int sts;

	if (CCP_get_tlm_cmd_discrimination(packet) == TLMCMD_CMD)
	{
		return PH_INVALID_DISCRIMINATOR;
	}
	for (i = 0; i < packet->length; i++)
		TlmTransferFrame.Packet.packet_data[i] = packet->data[i];

	TlmSend((int)((packet->length) - TLM_BYTE_COUNT));
	return PH_SUCCESS;
}

PH_ACK add_rt_tlm(CTCP* packet)
{
	unsigned char _dr_tlm = packet->data[CMD_POSITION_TLMCMD];
	if (msn_dr.enabled && _dr_tlm == (unsigned char)TLMCMD_TLM)
	{
		switch (CCP_get_tlm_id(packet))	//rt_tlm_list�ɂ͊܂߂Ȃ�tlm���`�F�b�N
		{
		case CCP_SF_DATA_ID:
			return PH_FORWARDED;
		case CCP_SUBCAM_DATA_ID:
			return PH_FORWARDED;
		}
	}

	//ASU���ʏ���
	if (CCP_get_tlm_id(packet) == CCP_AQU_TLM_ID)
	{
		AQU_TlmExtract(packet);
	}

	PL_ACK ack = PL_push_back(&rt_tlm_list, packet);
	if (ack != PL_SUCCESS) { 
#ifdef MSG_DEBUG
		Serial.println("ERROR: PH_PL_LIST_FULL"); 
#endif
		rt_tlm_list = PL_initialize(rt_tlm_stock_, RT_TLM_LIST_MAX);
		return PH_PL_LIST_FULL; 
	}

	return PH_REGISTERED;
}

static PH_ACK add_tl_cmd_(const CTCP* packet,
	size_t now)
{
	PL_ACK ack = PL_insert_tl_cmd(&tl_cmd_list, packet, now);
	switch (ack)
	{
	case PL_SUCCESS:
		return PH_TLC_REGISTERD;
	case PL_LIST_FULL:
		return PH_PL_LIST_FULL;
	case PL_TLC_PAST_TIME:
		return PH_TLC_PAST_TIME;
	case PL_TLC_ALREADY_EXISTS:
		return PH_TLC_ALREADY_EXISTS;
	default:
		return PH_UNKNOWN;
	}
}

static PH_ACK add_rt_cmd_(const CTCP* packet)
{
	PL_ACK ack = PL_push_back(&rt_cmd_list, packet);
	if (ack != PL_SUCCESS) { return PH_PL_LIST_FULL; }
	return PH_REGISTERED;
}

static PH_ACK add_adcs_cmd_(const CTCP* packet)
{
	PL_ACK ack = PL_push_back(&to_adcs_cmd_list, packet);
	if (ack != PL_SUCCESS) { return PH_PL_LIST_FULL; }
	return PH_REGISTERED;
}

static PH_ACK add_msn_cmd_(const CTCP* packet)
{
	PL_ACK ack = PL_push_back(&to_msn_cmd_list, packet);
	if (ack != PL_SUCCESS) { return PH_PL_LIST_FULL; }
	return PH_REGISTERED;
}

static PH_ACK add_fram_tlc(const CTCP* packet, utime_t now_utime)
{
	utime_t time = CCP_get_cmd_time(packet);
	if (now_utime > time)
		return PH_FRAM_TLC_PAST_TIME;
	if (fram_tlc_queue == FRAM_TLC_MAX)
		return PH_PL_LIST_FULL;

	if (FRAM_TLC_Write_packet(packet) != Success)
		return PH_FRAM_TLC_ALREADY_EXISTS;

	return PH_FRAM_TLC_REGISTERD;
}
