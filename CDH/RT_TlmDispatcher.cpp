// 
// 
// 

#include "RT_TlmDispatcher.h"

#include <string.h> // for memcmp

#include "utility.h"
#include "PacketHandler.h"
#include "cmd_analyze_obc.h"
#include "tlm_frame.h"
#include "BlockCommandDefinisions.h"
#include "STRX_update.h"

static void rt_tlm_transmitter_init_(void);
static void rt_tlm_transmitter_(void);

AppInfo rt_tlm_transmitter(void)
{
	return create_app_info("rttlm", 1, rt_tlm_transmitter_init_, rt_tlm_transmitter_);
}

static void rt_tlm_transmitter_init_(void)
{
	PL_clear_list(&rt_tlm_list);
}

static void rt_tlm_transmitter_(void)
{
	CTCP *tpkt = NULL;
	if (PL_is_empty(&rt_tlm_list)) { return; }

	tpkt = &(PL_get_head(&rt_tlm_list)->packet);

	if (PH_send_realtime_telemetry(tpkt) == PH_SUCCESS)
	{
		PL_drop_executed(&rt_tlm_list);
	}
}
