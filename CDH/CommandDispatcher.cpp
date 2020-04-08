// 
// 
// 

#include "CommandDispatcher.h"


#include "cmd_analyze_obc.h"
#include "TimeManager.h"
#include "PacketHandler.h"

static CDIS_EXEC_INFO CEI_init_(void);

CDIS CDIS_init(PL_Info* pli)
{
	CDIS cdis;

	// �R�}���h���s�����������B
	cdis.prev = CEI_init_();
	cdis.prev_err = CEI_init_();

	// ���s�G���[�J�E���^��0�ɏ������B
	cdis.error_counter = 0;

	// ���s���f�t���O�𖳌��ɐݒ�
	cdis.lockout = 0;

	// �ُ펞���s���f�t���O�𖳌��ɐݒ�B
	cdis.stop_on_error = 0;

	// �����ΏۂƂ���PacketList���N���A���ēo�^�B
	PL_clear_list(pli);
	cdis.pli = pli;

	return cdis;
}

static CDIS_EXEC_INFO CEI_init_(void)
{
	CDIS_EXEC_INFO cei;

	OBCT_clear(&cei.time);
	cei.time.step = 0;
	cei.code = 0;
	cei.sts = PH_SUCCESS;

	return cei;
}

void CDIS_dispatch_command(CDIS* cdis)
{
	// �p�P�b�g�R�s�[�p�B�T�C�Y���傫�����ߐÓI�ϐ��Ƃ��Đ錾�B
	static CTCP packet_;

	if (cdis->lockout)
	{
		// ���s�L���t���O������������Ă���ꍇ�͏����ł��؂�B
		return;
	}

	if (PL_is_empty(cdis->pli))
	{
		// ���s���ׂ��R�}���h�������ꍇ�͏����I��
		Serial.println("Invalid; CDIS");
		return;
	}

	if (cdis->prev.sts != PH_SUCCESS)
	{
		cdis->prev_err = cdis->prev;
	}

	// ���s���ׂ��R�}���h�p�P�b�g���擾�B
	packet_ = PL_get_head(cdis->pli)->packet;
	CCP_set_cmd_exe_type(&packet_, CCP_REALTIME); // TimeLine -> RealTime

												  // ���s�������L�^���R�}���h�����s�B
	cdis->prev.time = *master_clock;
	cdis->prev.code = CCP_get_cmd_id(&packet_);
	cdis->prev.sts = PH_despatch_command(&packet_);

	// ���s�����R�}���h�����X�g����j��
	PL_drop_executed(cdis->pli);

	if (cdis->prev.sts != PH_SUCCESS)
	{
		// ���s�����R�}���h�����s�ُ�X�e�[�^�X��Ԃ����ꍇ�B
		// �G���[�����J�E���^���J�E���g�A�b�v�B
		++(cdis->error_counter);

		if (cdis->stop_on_error == 1)
		{
			// �ُ펞���s���f�t���O���L���ȏꍇ�B
			// ���s���t���O�𖳌������ȍ~�̎��s�𒆒f�B
			cdis->lockout = 1;
		}
	}
}

void CDIS_clear_command_list(CDIS* cdis)
{
	// �ێ����Ă��郊�X�g�̓��e���N���A
	PL_clear_list(cdis->pli);
}

void CDIS_clear_error_status(CDIS* cdis)
{
	// ���s�G���[��Ԃ�������Ԃɕ���
	cdis->prev_err = CEI_init_();

	// �ώZ�G���[�񐔂�0�N���A
	cdis->error_counter = 0;
}
