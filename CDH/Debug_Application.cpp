// 
// 
// 

#include "Debug_Application.h"
#include "Temperature.h"

static cycle_t debug_interval_cycle;
void 	print_debug_message_(void);
void	_tlmcmd_debug_message(void);

AppInfo print_debug_message(void)
{
	return create_app_info("dbg_app", 30, NULL, print_debug_message_);
}

void print_debug_message_(void)
{
	static unsigned int c;

	if (c >= DEBUG_TASK_INTERVAL_SEC * OBCT_CYCLES_PER_SEC_IN_BC)
	{
#ifdef MSG_DEBUG
		DEBUG_Serial.println("\r\nTASUKI Satellite BUS:");
		DEBUG_Serial.print("clock: "); DEBUG_Serial.print(master_clock->master); DEBUG_Serial.print(", ");
		DEBUG_Serial.print("utime: "); DEBUG_Serial.print(master_clock->utime); DEBUG_Serial.print("\r\n");
		DEBUG_Serial.print("millis: "); DEBUG_Serial.print(millis()); DEBUG_Serial.print("\r\n");

		_tlmcmd_debug_message();
		EPS_DebugMessage();
		//GNSS_DebugMessage();
		STRX_DebugMessage();
		TempSens_DebugMessage();
		DEBUG_Serial.println("");
#endif
		c = 0;
	}
	c++;
}

void	_tlmcmd_debug_message(void)
{
	DEBUG_Serial.print("TlmCount[01]: "); DEBUG_Serial.print(TlmCount[TLM_ID_01]); DEBUG_Serial.println(", ");

	DEBUG_Serial.print("GS : cmdCount: "); DEBUG_Serial.print(GsCmdCount); DEBUG_Serial.print(", ");
	DEBUG_Serial.print("CmdCode: "); DEBUG_Serial.print(GsCmdCode); DEBUG_Serial.print(", ");
	DEBUG_Serial.print("CmdErrorSts: "); DEBUG_Serial.print(GsCmdErrorSts); DEBUG_Serial.print(", ");
	DEBUG_Serial.print("CmdSts: "); DEBUG_Serial.print(GsCmdSts); DEBUG_Serial.print("\r\n");

	DEBUG_Serial.print("CDH: cmdCount: "); DEBUG_Serial.print(CdhCmdCount); DEBUG_Serial.print(", ");
	DEBUG_Serial.print("CmdCode: "); DEBUG_Serial.print(CdhCmdCode); DEBUG_Serial.println("");

	DEBUG_Serial.print("RT_TLM: "); DEBUG_Serial.print(rt_tlm_list.pl_public.active_nodes); DEBUG_Serial.print("/");  DEBUG_Serial.print(rt_tlm_list.pl_public.total_nodes);
	DEBUG_Serial.println("");

	DEBUG_Serial.println("");

}