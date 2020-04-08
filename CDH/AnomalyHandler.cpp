// 
// 
// 

#include "AnomalyHandler.h"
#include "utility.h"
#include "TimeManager.h"
#include "UVC_app.h"
#include "BlockCommandDefinisions.h"
#include "PacketHandler.h"
#include "cmd_analyze_obc.h"

static AHInfo ahi_;
const AHInfo *ahi;

static size_t prev_pos_;
static size_t prev_act_;

static void AH_init_(void);
static void load_default_rules_(void);
static void AH_exec_(void);
static size_t check_rules_(size_t record_id);
static int is_equal_code_(const AnomalyCode* a,
	const AnomalyCode* b);
static void respond_to_anomaly_(size_t id);
static void add_rule_(size_t id,
	const AHRule *ahr);
static void AH_clear_log_(void);

AppInfo AH_create_app(void)
{
	return create_app_info("ah", 1, AH_init_, AH_exec_);
}

static void AH_init_(void)
{
	int i;

	ahi = &ahi_;
	// �����ݒ�͂��ׂẴ��[���𖳌�������
	for (i = 0; i<AH_MAX_RULES; ++i) { ahi_.elements[i].is_active = 0; }
	// �f�t�H���g�̃��[���\����ǂݍ���
	load_default_rules_();
	// �Ή��������[���N���A
	OBCT_clear(&(ahi_.respond_at));
	// �����͔͈͊O�ɐݒ肵�Y���Ȃ���\��
	ahi_.latest_id = AH_MAX_RULES;
	// �Ή����s�����N���A
	ahi_.action_counter = 0;
	// �A�m�}���Ή����m�p�̕ϐ����N���A
	prev_act_ = 0;
	// �A�m�}�������󋵂�������
	AH_clear_log_();
}

static void load_default_rules_(void)
{
	
	AHRule ahr;

	// Under Voltage Control Lv.1
	ahr.code.group = AL_UVC;
	ahr.code.local = UVC_LEVEL1;
	ahr.cond = AH_SINGLE;
	ahr.threshold = 0;
	ahr.bc_id = BC_UVC_LV1_ACTION;

	add_rule_(0, &ahr);
	ahi_.elements[0].is_active = 1;

	// Under Voltage Control Lv.2
	ahr.code.group = AL_UVC;
	ahr.code.local = UVC_LEVEL2;
	ahr.cond = AH_SINGLE;
	ahr.threshold = 0;
	ahr.bc_id = BC_UVC_LV2_ACTION;

	add_rule_(1, &ahr);
	ahi_.elements[1].is_active = 1;

	// Under Voltage Control Lv.3
	ahr.code.group = AL_UVC;
	ahr.code.local = UVC_LEVEL3;
	ahr.cond = AH_SINGLE;
	ahr.threshold = 0;
	ahr.bc_id = BC_UVC_LV3_ACTION;

	add_rule_(2, &ahr);
	ahi_.elements[2].is_active = 1;

}

static void AH_exec_(void)
{
	// �A�m�}�������ω����Ă��Ȃ��ꍇ
	if (ahi_.al_pointer.count == al->counter) { return; }
	// �V���ȃA�m�}�������������ꍇ
	else if (ahi_.al_pointer.count < al->counter)
	{
		size_t record_id;
		for (record_id = ahi_.al_pointer.pos; record_id<al->header; ++record_id)
		{
			size_t id = check_rules_(record_id);
			if (id != AH_MAX_RULES) { respond_to_anomaly_(id); }
		}
		// �����I����A�ŐV�ʒu�E�񐔂�AnomalyLogger���Ɠ���
		ahi_.al_pointer.pos = al->header - 1;
		ahi_.al_pointer.count = al->counter;
	}
	// �ُ�: AnomalyHandler��AnomalyLogger�̊֌W������Ă���B
	else
	{
		// �d�����Ȃ��̂ŁA�L�^�����Z�b�g��������������B
		AH_clear_log_();
	}
}

static size_t check_rules_(size_t record_id)
{
	const AnomalyRecord* ar = AL_get_record(record_id);
	int is_latest = (record_id == al->header);
	size_t id;
	// �S���[������������
	for (id = 0; id<AH_MAX_RULES; ++id)
	{
		AHElement ahe = ahi_.elements[id];
		// ���[���������̏ꍇ�̓X�L�b�v
		if (!(ahe.is_active)) { continue; }
		// �R�[�h�s��v�̏ꍇ�̓X�L�b�v
		else if (!is_equal_code_(&(ahe.rule.code), &(ar->code))) { continue; }

		// �R�[�h�ɑΉ����郋�[�������݁B���������]���B
		switch (ahe.rule.cond)
		{
		case AH_SINGLE:
			// �R�[�h�����݂������_�Ŗ��������v�B
			return id;
		case AH_CONTINUOUS:
			// �A���񐔂𒴉߂����ꍇ�ɏ������v�B
			if (ahe.rule.threshold < ar->run_length) { return id; }
			break;
		case AH_CUMULATE:
			// �ώZ�񐔂𒴉߂����ꍇ�ɏ������v�B
			if (ahe.counter <= ar->run_length) { return id; }
			// �ŐV���R�[�h�ɑ΂����r�łȂ��ꍇ�͐ώZ�����X�V����B
			else if (!is_latest) { ahe.counter -= ar->run_length; }
			break;
		}
	}
	return AH_MAX_RULES; // �Y���Ȃ��̏ꍇ�͓o�^�����Ԃ��B
}

static int is_equal_code_(const AnomalyCode* a,
	const AnomalyCode* b)
{
	return ((a->group == b->group) && (a->local == b->local));
}

static void respond_to_anomaly_(size_t id)
{
	// �Ή��u���b�N�R�}���h�����A���^�C���R�}���h�œW�J
	unsigned char param = (unsigned char)ahi_.elements[id].rule.bc_id;
	CTCP packet = CCP_form_rt_cmd(CODE_Cmd_DEPLOY_BLOCK, &param, 1u);
	PH_despatch_command(&packet);

	// ���s�������[�����L�^���񐔂��X�V
	ahi_.respond_at = *master_clock;
	ahi_.latest_id = id;
	++ahi_.action_counter;

	// ���v�������[���͖���������B
	ahi_.elements[id].is_active = 0;
}

AppInfo print_ah_status(void)
{
	return create_app_info("ahs", 5, NULL, print_ah_status_);
}

void print_ah_status_(void)
{
	/*
	Printf("AH: ID %d, ACT %d, AT <%d, %d, %d>, POS %d, CTR %d\n",
		ahi_.latest_id, ahi_.action_counter,
		ahi_.respond_at.master, ahi_.respond_at.mode, ahi_.respond_at.step,
		ahi_.al_pointer.pos, ahi_.al_pointer.count);
	*/
	// �V��̃A�m�}�������������ꍇ�x����1��
	if (prev_pos_ != ahi_.al_pointer.pos)
	{
		DEBUG_Serial.println("\a");
		prev_pos_ = ahi_.al_pointer.pos;
	}

	// �A�m�}���Ή������������ꍇ�x����2��
	if (prev_act_ != ahi_.action_counter)
	{
		DEBUG_Serial.println("\a\a");
		prev_act_ = ahi_.action_counter;
	}
}

int Cmd_AH_register_rule(const CTCP* packet)
{
	enum { ID, GROUP, LOCAL, COND, THRESHOLD, BC };
	const unsigned char* param = CCP_get_cmd_param(packet);
	AHRule ahr;

	if (CCP_get_length(packet) != 11 + 6)
	{
		// �p�����[�^��6Bytes
		return CMD_ERROR_ILLEGAL_LENGTH;
	}
	else if (param[ID] >= AH_MAX_RULES)
	{
		// �o�^�w��ʒu�����e�͈͊O
		return CMD_ERROR_ILLEGAL_PARAMETER;
	}
	else if (param[COND] > AH_CUMULATE)
	{
		// �����������`���ꂽ���̂ƈ�v���Ȃ�
		return CMD_ERROR_ILLEGAL_PARAMETER;
	}

	ahr.code.group = param[GROUP];
	ahr.code.local = param[LOCAL];
	ahr.cond = (AHCondition)param[COND];
	ahr.threshold = param[THRESHOLD];
	ahr.bc_id = param[BC];

	add_rule_((size_t)param[ID], &ahr);

	return CMD_ERROR_NONE;
}

static void add_rule_(size_t id,
	const AHRule *ahr)
{
	ahi_.elements[id].is_active = 0; // �o�^���_�ł͖����Ƃ���
	ahi_.elements[id].rule = *ahr;
	ahi_.elements[id].counter = ahr->threshold;
}

int Cmd_AH_activate_rule(const CTCP* packet)
{
	const unsigned char* param = CCP_get_cmd_param(packet);

	if (CCP_get_length(packet) != 11 + 1)
	{
		// �p�����[�^��1Byte
		return CMD_ERROR_ILLEGAL_LENGTH;
	}
	else if (param[0] >= AH_MAX_RULES)
	{
		// �w��ʒu���͈͊O
		return CMD_ERROR_ILLEGAL_PARAMETER;
	}

	ahi_.elements[param[0]].is_active = 1;
	ahi_.elements[param[0]].counter = ahi_.elements[param[0]].rule.threshold;
	return CMD_ERROR_NONE;
}

int Cmd_AH_inactivate_rule(const CTCP* packet)
{
	const unsigned char* param = CCP_get_cmd_param(packet);

	if (CCP_get_length(packet) != 11 + 1)
	{
		// �p�����[�^��1Byte
		return CMD_ERROR_ILLEGAL_LENGTH;
	}
	else if (param[0] >= AH_MAX_RULES)
	{
		// �w��ʒu���͈͊O
		return CMD_ERROR_ILLEGAL_PARAMETER;
	}

	ahi_.elements[param[0]].is_active = 0;
	return CMD_ERROR_NONE;
}

int Cmd_AH_clear_log(const CTCP* packet)
{
	if (CCP_get_length(packet) != 11)
	{
		// �p�����[�^�Ȃ�
		return CMD_ERROR_ILLEGAL_LENGTH;
	}

	AH_clear_log_();
	return CMD_ERROR_NONE;
}

static void AH_clear_log_(void)
{
	// AnomalyHnadler���̕ێ�����������
	ahi_.al_pointer.count = 0;
	ahi_.al_pointer.pos = 0;
	// �A�m�}���������m�p�ϐ���������
	prev_pos_ = 0;
	// AnomalyLogger�̃��X�g���N���A
	AL_clear();
}

