// 
// 
// 

#include "AnomalyLogger.h"
#include <string.h> // memset
#include "TimeManager.h"

static AnomalyLogger al_;
const AnomalyLogger* al;

static void AL_clear_records_(void);
static int AC_is_equal_(const AnomalyCode* lhs,
	const AnomalyCode* rhs);

void AL_initialize(void)
{
	AL_clear();
	al = &al_;
}

void AL_add_anomaly(unsigned int group,
	unsigned int local)
{
	const AnomalyRecord* prev = AL_get_latest_record();
	AnomalyRecord al_new;
	al_new.time = TMGR_get_time();
	al_new.code.group = group;
	al_new.code.local = local;
	al_new.run_length = 1;

	// �o�^������̏ꍇ�͂��̎������L�^����
	if (al_.header == AL_RECORD_MAX - 1)
	{
		al_new.code.group = AL_ANOMALY_LOGGER;
		al_new.code.local = AL_FULL;
	}

	// ����AnomalyCode�̓o�^�͈��k����
	if (AC_is_equal_(&(al_new.code), &(prev->code)))
	{
		// �����ƘA�����X�V���ēo�^
		al_new.run_length += prev->run_length;
		al_.records[al_.header - 1] = al_new;
	}
	else
	{
		// �V�K�o�^
		al_.records[al_.header] = al_new;
		// �o�^��������łȂ��Ȃ�擪�ʒu���C���N�������g
		if (al_.header != AL_RECORD_MAX - 1) { ++al_.header; }
	}

	++al_.counter;
}

size_t AL_get_counter(void)
{
	return al_.counter;
}

size_t AL_get_header(void)
{
	return al_.header;
}

const AnomalyRecord* AL_get_record(size_t pos)
{
	// �w��͈̓G���[�̏ꍇ��NULL��Ԃ�
	if (pos >= al_.header) { return NULL; }

	return &(al_.records[pos]);
}

const AnomalyRecord* AL_get_latest_record(void)
{
	// �o�^�A�m�}���[���Ȃ��ꍇ�͐擪�v�f��Ԃ��B
	// ���̏ꍇ�̐擪�v�f��AL_initialize()�ɂ���ă[���N���A�ς݁B
	if (al_.header == 0) { return &(al_.records[0]); }

	return &(al_.records[al_.header - 1]);
}

void AL_clear(void)
{
	al_.counter = 0;
	al_.header = 0;
	AL_clear_records_();
}

static void AL_clear_records_(void)
{
	int i;
	AnomalyRecord ar = { { 0, 0, 0 },{ 0, 0 }, 0 };
	for (i = 0; i<AL_RECORD_MAX; ++i) { al_.records[i] = ar; }
}

static int AC_is_equal_(const AnomalyCode* lhs,
	const AnomalyCode* rhs)
{
	return ((lhs->group == rhs->group) && (lhs->local == rhs->local));
}
