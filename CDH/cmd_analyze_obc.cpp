#include "cmd_action.h"
#include "cmd_analyze_obc.h"
#include "TL_CmdDispatcher.h"

int       CmdSts;
int       CmdCode;
int       CmdCount;
int       CmdErrorSts;
int       CmdLength;
int       CmdTo;
int       CmdTlmCmd;


int cmdExec(const CTCP* packet)
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


	if (CmdCode == CODE_Cmd_NOP) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_NOP(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_DEPLOY_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_DEPLOY_BLOCK(packet);	// �����ł��������u���b�N�̃u���b�N�R�}���h��W�J���ă^�C�����C���R�}���h�Ɋi�[����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_CLEAR_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_CLEAR_BLOCK(packet);	// �����ŗ^�����u���b�N�̃u���b�N�R�}���h���N���A����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SET_BLOCK_POSITION) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_BLOCK_POSITION(packet);	// �u���b�N�R�}���h�̓o�^�ʒu���w�肷��B
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_ACTIVATE_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_ACTIVATE_BLOCK(packet);	// �u���b�N�R�}���h��L��������
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FINISH_TRANSITION) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FINISH_TRANSITION(packet);	// ���[�h�J�ڏI���g���K�R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SET_TRANSITION_TABLE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_TRANSITION_TABLE(packet);	// TransitionTable���������R�}���h(from, to , index)
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_EXECUTE_APP) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_EXECUTE_APP(packet);	// �A�v���P�[�V�������s�R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_TDSP_SET_TASK_LIST_ID) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_TDSP_SET_TASK_LIST_ID(packet);	// �^�X�N���X�gBC�ݒ�R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_ROTATE_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_ROTATE_BLOCK(packet);	// BC��]�R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_COMBINE_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_COMBINE_BLOCK(packet);	// BC�Z���R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AH_register_rule) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_register_rule(packet);	// �A�m�}�����[���ݒ�R�}���h(id, group)
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AH_activate_rule) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_activate_rule(packet);	// �A�m�}�����[���L�����R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AH_inactivate_rule) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_inactivate_rule(packet);	// �A�m�}�����[���������R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AH_clear_log) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AH_clear_log(packet);	// �A�m�}�����O�N���A�R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AS_set_id) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AS_set_id(packet);	// �A�v��ID�ݒ�R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_REGISTER_APP) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_REGISTER_APP(packet);	// �A�v���o�^�R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_INITIALIZE_APP) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_INITIALIZE_APP(packet);	// �A�v���������R�}���h
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SET_MODE_LIST) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_MODE_LIST(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_START_TRANSITION) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_START_TRANSITION(packet);	// �^�p���[�h��ύX����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_TlmRequest) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_TlmRequest(packet);	// �w��TLM_ID�̃e�����g�����_�E�������N����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_TLM_SET_MODE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_TLM_SET_MODE(packet);	// OBC�e�����g�����[�h�ݒ�F�J�n
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SET_TLM_INTERVAL) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_TLM_INTERVAL(packet);	// interval[s]
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_CLEAR_ALL_TIMELINE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_CLEAR_ALL_TIMELINE(packet);	// MOBC�R�}���h�N���A(�^�C�����C��)
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_CLEAR_TIMELINE_AT) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_CLEAR_TIMELINE_AT(packet);	// MOBC�ɓo�^����Ă���^�C�����C���R�}���h����1�I��ō폜����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SET_OBC_TIME) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_OBC_TIME(packet);	// MOBC�����^�O����͒l�ɐݒ肷��
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_STRX_SET_BR) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_STRX_SET_BR(packet);	// param0: 0=4kbps, 1:32kbps, 2:64kbps (IF���STRX�{�̓����ɕύX)
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_STRX_TX_ON) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_STRX_TX_ON(packet);	// param0: 0=OFF, 1=ON
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MUX_UART_SET) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MUX_UART_SET(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SYNC_TIME) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SYNC_TIME(packet);	// �n��ǎ����Ɖq��(OBC, RTC)�����𓯊�������
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SYNC_OBC_TIME) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SYNC_OBC_TIME(packet);	// �n��ǎ�����OBC�����𓯊�������
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SYNC_RTC_TIME) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SYNC_RTC_TIME(packet);	// �n��ǎ�����RTC�����𓯊�������
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SYNC_GPS_TIME) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SYNC_GPS_TIME(packet);	// GPS�����Ɖq�������𓯊�������
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_SYNC_RTC_ENABLE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_SYNC_RTC_ENABLE(packet);	// OBC���Z�b�g����RTC�Ǝ�������������(ENA:1, DIS:0)
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_TLC_CLEAR_ALL) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_TLC_CLEAR_ALL(packet);	// OBC-FRAM�̃R�}���h�N���A(�^�C�����C��)
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_TLC_CLEAR_AT_TIME) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_TLC_CLEAR_AT_TIME(packet);	// OBC-FRAM�ɓo�^����Ă���^�C�����C���R�}���h���玞���w���1�I��ō폜����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_TLC_CLEAR_AT_NUMBER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_TLC_CLEAR_AT_NUMBER(packet);	// OBC-FRAM�ɓo�^����Ă���^�C�����C���R�}���h����ԍ��w���1�I��ō폜����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SOFTWARE_RESET) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SOFTWARE_RESET(packet);	// �\�t�g�������Z�b�g
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_INITIALIZE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_INITIALIZE(packet);	// FRAM�̏�����, �ł��グ�O���[�h�ɂ���
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_CMD_INITIALIZE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_CMD_INITIALIZE(packet);	// FRAM(TLC, BC)������������(�S����)
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_DEPLOY_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_DEPLOY_BLOCK(packet);	// �����ł��������u���b�N�̃u���b�N�R�}���h��W�J���ă^�C�����C���R�}���h�Ɋi�[����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_REMOVE_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_REMOVE_BLOCK(packet);	// �����ŗ^�����u���b�N�̃u���b�N�R�}���h���N���A����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_SET_BLOCK_POSITION) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_SET_BLOCK_POSITION(packet);	// �u���b�N�R�}���h�̓o�^�ʒu���w�肷��B
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_ACTIVATE_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_ACTIVATE_BLOCK(packet);	// �u���b�N�R�}���h�̗L���ɂ���
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_REMOVE_BLOCK_AT_POSITION) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_REMOVE_BLOCK_AT_POSITION(packet);	// �w��ID�̈ʒu�̃u���b�N�R�}���h����������
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_REMOVE_ALL_BLOCK) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_REMOVE_ALL_BLOCK(packet);	// �o�^�ς݂̃u���b�N�R�}���h�̏������ׂď���
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_REQUEST_BC_INFO) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_REQUEST_BC_INFO(packet);	// �w��ID�̃u���b�N�R�}���h�����_�E�������N����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_FRAM_REQUEST_BC_PARAM) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_FRAM_REQUEST_BC_PARAM(packet);	// �w��ID, �ʒu�̃u���b�N�R�}���h�����_�E�������N����
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_STRX_SET_TX_PWR) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_STRX_SET_TX_PWR(packet);	// Power Mode�̐؂�ւ�
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_STRX_OTC_ENABLE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_STRX_OTC_ENABLE(packet);	// 0: DIS, 1:EN
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_STRX_OTC_SET_THRESHOLD) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_STRX_OTC_SET_THRESHOLD(packet);	// Temperature[degC]
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SET_STX_ONOFF_DURATION) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_STX_ONOFF_DURATION(packet);	// param1: on����[s]�Cparam2: off����[s]
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SET_STX_BURST_DURATION) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SET_STX_BURST_DURATION(packet);	// �A�����M���Ԃ�ݒ�[s]
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_EPS_RESET) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_EPS_RESET(packet);	// 0: MAIN, 1:SUB
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_EPS_UVC_ENABLE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_EPS_UVC_ENABLE(packet);	// 0: DIS, 1:EN
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_EPS_UVC_SET_BAT_V) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_EPS_UVC_SET_BAT_V(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_EPS_SELECT) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_EPS_SELECT(packet);	// 0:ChA, 1:ChB
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_ADCS_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_ADCS_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_RW_ENABLE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_RW_ENABLE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_HEATER_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_HEATER_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_RW_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_RW_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MSN_unreg_ENABLE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_unreg_ENABLE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MSN_5V_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_5V_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SUBCAM_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SUBCAM_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_SF_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_SF_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MTQ_POWER_OBC) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MTQ_POWER_OBC(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_POWER_SAFE_OFF) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_POWER_SAFE_OFF(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_UNREG_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_UNREG_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_POWER_REBOOT) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_POWER_REBOOT(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_MTQ_X) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_MTQ_X(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_MTQ_Y) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_MTQ_Y(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_MTQ_Z) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_MTQ_Z(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AQU_5V_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_AQU_5V_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OVCO_5V_POWER) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OVCO_5V_POWER(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_DR_RECORD) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_DR_RECORD(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_DR_REPLAY_PARAM_SET) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_DR_REPLAY_PARAM_SET(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_DR_REPLAY) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_DR_REPLAY(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_DR_ERASE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_DR_ERASE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MSN_DR_RECORD_ENABLE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_DR_RECORD_ENABLE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MSN_DR_REPLAY_PARAM_SET) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_DR_REPLAY_PARAM_SET(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MSN_DR_REPLAY) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_DR_REPLAY(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MSN_DR_ERASE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_DR_ERASE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_OBC_DR_INITIALIZE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_OBC_DR_INITIALIZE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_MSN_DR_INITIALIZE) {
		CmdSts = CMD_STATUS_START;
		CmdErrorSts = Cmd_MSN_DR_INITIALIZE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AQU_ANOMALY_ENABLE) {
		CmdSts = CMD_STATUS_START;
		//CmdErrorSts = Cmd_AQU_ANOMALY_ENABLE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AQU_ANOMALY_SET_THR_DIFF_PRESSURE) {
		CmdSts = CMD_STATUS_START;
		////CmdErrorSts = Cmd_AQU_ANOMALY_SET_THR_DIFF_PRESSURE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AQU_ANOMALY_SET_THR_TEMPERATURE) {
		CmdSts = CMD_STATUS_START;
		//CmdErrorSts = Cmd_AQU_ANOMALY_SET_THR_TEMPERATURE(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AQU_ANOMALY_SET_THR_RV_OPEN_TIME) {
		CmdSts = CMD_STATUS_START;
		//CmdErrorSts = Cmd_AQU_ANOMALY_SET_THR_RV_OPEN_TIME(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}
	}
	else if (CmdCode == CODE_Cmd_AQU_ANOMALY_SET_THR_TV_OPEN_TIME) {
		CmdSts = CMD_STATUS_START;
		//CmdErrorSts = Cmd_AQU_ANOMALY_SET_THR_TV_OPEN_TIME(packet);	// 
		if (CmdErrorSts == CMD_ERROR_NONE) {
			CmdSts = CMD_STATUS_END;
		}
		else {
			CmdSts = CMD_STATUS_ERROR_EXE;
		}


		//##//##//##//##//##//##//##//##//##//##//##//##//##//##//##//##
		/*
		This pattern is a "separator".
		This should not be changed.
		This should not be used in other places.
		*/

	}
	else {
		Serial.println("*** UNKONWN CMD ***");
		return(CMD_ERROR_NOT_DEFINED);
	}

	return(CmdErrorSts);
}
