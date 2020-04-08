// 
// 
// 

#include "NormalSL.h"

#include "BlockCommandTable.h"
#include "cmd_item.h"

void BC_load_any_to_normal(size_t pos)
{
	CTCP temp;

	BCT_clear_block(pos);

	temp = adcs_on();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(2));
	BCT_register_cmd(&temp);
	
	temp = cmd_power_reboot();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(4));
	BCT_register_cmd(&temp);

	temp = cmd_stx_power(0x0F);
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(8));
	BCT_register_cmd(&temp);

	temp = finish_transition();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(10));
	BCT_register_cmd(&temp);

	BCT_activate_block();
}
