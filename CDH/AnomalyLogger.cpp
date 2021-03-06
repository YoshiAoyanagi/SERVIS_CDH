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

	// 登録数上限の場合はその事実を記録する
	if (al_.header == AL_RECORD_MAX - 1)
	{
		al_new.code.group = AL_ANOMALY_LOGGER;
		al_new.code.local = AL_FULL;
	}

	// 同じAnomalyCodeの登録は圧縮する
	if (AC_is_equal_(&(al_new.code), &(prev->code)))
	{
		// 時刻と連長を更新し再登録
		al_new.run_length += prev->run_length;
		al_.records[al_.header - 1] = al_new;
	}
	else
	{
		// 新規登録
		al_.records[al_.header] = al_new;
		// 登録数が上限でないなら先頭位置をインクリメント
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
	// 指定範囲エラーの場合はNULLを返す
	if (pos >= al_.header) { return NULL; }

	return &(al_.records[pos]);
}

const AnomalyRecord* AL_get_latest_record(void)
{
	// 登録アノマリーがない場合は先頭要素を返す。
	// この場合の先頭要素はAL_initialize()によってゼロクリア済み。
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
