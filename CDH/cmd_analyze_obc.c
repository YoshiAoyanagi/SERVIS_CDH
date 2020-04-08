##include "cmd_action.h"
#include "cmd_analyze.h"

int       CmdSts;
int       CmdCode;
int       CmdCount;
int       CmdErrorSts;
int       CmdLength;
int       CmdTo;
int       CmdTlmCmd;


int cmdExec(const CTCP *packet)
{
	CmdCode = CCP_get_cmd_id(packet);
	CmdCount++;
	CmdCount &= 0xff;

/*
This pattern is a "separator".
This should not be changed.
This should not be used in other places.
*/
//##//##//##//##//##//##//##//##//##//##//##//##//##//##//##//##


	if(CmdCode == CODE_Cmd_NOP){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_NOP(packet);	// �_�~�[�R�}���h, �R�}���h�J�E���g�̂ݏ㏸���邪�����N����Ȃ�
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_DEPLOY_BLOCK){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_DEPLOY_BLOCK(packet);	// �����ł��������u���b�N�̃u���b�N�R�}���h��W�J���ă^�C�����C���R�}���h�Ɋi�[����
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_CLEAR_BLOCK){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_CLEAR_BLOCK(packet);	// �����ŗ^�����u���b�N�̃u���b�N�R�}���h���N���A����
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_BLOCK_POSITION){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_BLOCK_POSITION(packet);	// �u���b�N�R�}���h�̓o�^�ʒu���w�肷��B
param1: �u���b�N�ԍ� [0-7]
param2: �R�}���h�ʒu [0-7]
�ő�8�̃R�}���h��1�̃u���b�N�R�}���h�ɓo�^���\�ł��̂悤�ȃu���b�N�R�}���h���ő�8��`�ł���
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_ACTIVATE_BLOCK){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_ACTIVATE_BLOCK(packet);	// �u���b�N�R�}���h��L��������

		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_FINISH_TRANSITION){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FINISH_TRANSITION(packet);	// ���[�h�J�ڏI���g���K�R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_TRANSITION_TABLE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_TRANSITION_TABLE(packet);	// TransitionTable���������R�}���h(from, to , index)
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_EXECUTE_APP){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_EXECUTE_APP(packet);	// �A�v���P�[�V�������s�R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_TASK_LIST_ID){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_TASK_LIST_ID(packet);	// �^�X�N���X�gBC�ݒ�R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_ROTATE_BLOCK){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_ROTATE_BLOCK(packet);	// BC��]�R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_COMBINE_BLOCK){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_COMBINE_BLOCK(packet);	// BC�Z���R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_AH_register_rule){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_register_rule(packet);	// �A�m�}�����[���ݒ�R�}���h(id, group)
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_AH_activate_rule){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_activate_rule(packet);	// �A�m�}�����[���L�����R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_AH_inactivate_rule){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_inactivate_rule(packet);	// �A�m�}�����[���������R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_AH_clear_log){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_clear_log(packet);	// �A�m�}�����O�N���A�R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_AS_set_id){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AS_set_id(packet);	// �A�v��ID�ݒ�R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_CMD_REGISTER_APP){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = CMD_REGISTER_APP(packet);	// �A�v���o�^�R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_CMD_INITIALIZE_APP){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = CMD_INITIALIZE_APP(packet);	// �A�v���������R�}���h
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_MODE_LIST){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_MODE_LIST(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_START_TRANSITION){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_START_TRANSITION(packet);	// �^�p���[�h��ύX����
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_GENERATE_TLM){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_GENERATE_TLM(packet);	// Param1[1byte]:TLM_ID
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_TLM_SET_MODE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_TLM_SET_MODE(packet);	// OBC�e�����g�����[�h�ݒ�F�J�n
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_TLM_INTERVAL){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_TLM_INTERVAL(packet);	// interval[s]
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_DR_INTERVAL){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_DR_INTERVAL(packet);	// interval[s]
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_CLEAR_ALL_TIMELINE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_CLEAR_ALL_TIMELINE(packet);	// MOBC�R�}���h�N���A(�^�C�����C��)
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_CLEAR_TIMELINE_AT){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_CLEAR_TIMELINE_AT(packet);	// MOBC�R�}���h�N���A(�^�C�����C��)
MOBC�ɓo�^����Ă���^�C�����C���R�}���h����1�I��ō폜����
�o�^����Ă��鎞��(MOBC_TLM_TIME)���p�����[�^�Ŏw�肷��
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_MOBC_TIME){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_MOBC_TIME(packet);	// MOBC�����^�O����͒l�ɐݒ肷��
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_MSN_DATA_FORWARD){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_DATA_FORWARD(packet);	// param1: target id
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_DR_RECORD_MTLM){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_DR_RECORD_MTLM(packet);	// param1: start/stop, param2: interval[s]
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_DR_REPLAY_SET){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_DR_REPLAY_SET(packet);	// param1: start sector, param2: start pk-address
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_DR_REPLAY){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_DR_REPLAY(packet);	// DR���v���C�I��
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_DR_ERASE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_DR_ERASE(packet);	// DR����: S&F, param2: num sector
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_COM_MUX_UART_SET){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = COM_MUX_UART_SET(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_SOFTWARE_RESET){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = SOFTWARE_RESET(packet);	// �\�t�g�������Z�b�g
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_FRAM_INITIALIZE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = FRAM_INITIALIZE(packet);	// FRAM�̏�����, �ł��グ�O���[�h�ɂ���
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_STX_POWER_MODE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_STX_POWER_MODE(packet);	// Power Mode�̐؂�ւ�
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_STX_ON){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_STX_ON(packet);	// ���M�@�̑��M�@�\��ON�ɂ���
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_STX_OFF){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_STX_OFF(packet);	// ���M�@�̑��M�@�\��OFF�ɂ���
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_Cmd_SET_DOWNLINK_BITRATE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_DOWNLINK_BITRATE(packet);	// �_�E�������N���x���ꊇ�Őݒ肷��
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_EPS_RESET){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = EPS_RESET(packet);	// 0: MAIN, 1:SUB
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_EPS_UVC_ENABLE){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = EPS_UVC_ENABLE(packet);	// 0: DIS, 1:EN
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_EPS_UVC_SET_BAT_V){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = EPS_UVC_SET_BAT_V(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_ADCS_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = ADCS_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_RW_unreg_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = RW_unreg_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_RESERVE_5V_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = RESERVE_5V_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_GNSSR_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = GNSSR_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_MSN_unreg_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = MSN_unreg_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_MSN_5V_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = MSN_5V_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_SUBCAM_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = SUBCAM_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_SF_POWER){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = SF_POWER(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_SAS_POWER_TO_ADCS){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = SAS_POWER_TO_ADCS(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_MTQ_POWER_TO_ADCS){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = MTQ_POWER_TO_ADCS(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}else if(CmdCode == CODE_RW_5V_POWER_TO_ADCS){
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = RW_5V_POWER_TO_ADCS(packet);	// 
		if(CmdErrorSts == CMD_ERROR_NONE){
			CmdSts = CMD_STATUS_END;
		}else{
			CmdSts = CMD_STATUS_ERROR_EXE;
		}


//##//##//##//##//##//##//##//##//##//##//##//##//##//##//##//##
/*
This pattern is a "separator".
This should not be changed.
This should not be used in other places.
*/

	}else{
		Printf("*** UNKONWN CMD ***");
		Printf("%4X\n", CmdCode);
		return(CMD_ERROR_NOT_DEFINED);
	}
	
	return(CmdErrorSts);
}
