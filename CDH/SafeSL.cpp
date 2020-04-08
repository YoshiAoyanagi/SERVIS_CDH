// 
// 
// 

#include "SafeSL.h"


#include "BlockCommandTable.h"
#include "cmd_item.h"

void BC_load_any_to_safe(size_t pos)
{
	CTCP temp;

	BCT_clear_block(pos);

	temp = adcs_mode_standby();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(2));
	BCT_register_cmd(&temp);

	temp = cmd_stx_bitrate(0x00);
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(4));
	BCT_register_cmd(&temp);

	temp = adcs_off();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(6));
	BCT_register_cmd(&temp);

	temp = cmd_stx_power(0x0F);
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(10));
	BCT_register_cmd(&temp);

	temp = pwr_off_safe();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(12));
	BCT_register_cmd(&temp);

	temp = finish_transition();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(14));
	BCT_register_cmd(&temp);

	BCT_activate_block();
}
