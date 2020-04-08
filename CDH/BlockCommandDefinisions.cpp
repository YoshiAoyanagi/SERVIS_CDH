// 
// 
// 

#include "BlockCommandDefinisions.h"


#include "BlockCommandTable.h"

static void BC_load_uvc_lv1_action(size_t pos);
static void BC_load_uvc_lv2_action(size_t pos);
static void BC_load_uvc_lv3_action(size_t pos);

void BC_load_defaults(void)
{
	// Block Cmds for TaskList
	BC_load_normal_mode(BC_TL_NORMAL);
	BC_load_tlmcmd_task(BC_AR_TLMCMD);
	BC_load_peripheral_task(BC_AR_PERIPHERAL);

	// Block Cmds for SequenceList
	BC_load_any_to_safe(BC_SL_ANY_TO_SAFE);
	BC_load_any_to_normal(BC_SL_ANY_TO_NORMAL);
	BC_load_any_to_initial(BC_SL_ANY_TO_INITIAL);

	BC_load_uvc_lv1_action(BC_UVC_LV1_ACTION);
	BC_load_uvc_lv2_action(BC_UVC_LV2_ACTION);
	BC_load_uvc_lv3_action(BC_UVC_LV3_ACTION);
}


static void BC_load_uvc_lv1_action(size_t pos)	//ミッション機器OFF
{
	CTCP temp;

	BCT_clear_block(pos);

	temp = sf_off();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(2));
	BCT_register_cmd(&temp);

	temp = subcam_off();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(4));
	BCT_register_cmd(&temp);

	temp = msn5v_off();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(6));
	BCT_register_cmd(&temp);

	BCT_activate_block();
}

static void BC_load_uvc_lv2_action(size_t pos)	//Safe mode
{
	CTCP temp;

	BCT_clear_block(pos);

	temp = start_transition(SAFE);
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(2));
	BCT_register_cmd(&temp);

	temp = cmd_stx_off();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(4));
	BCT_register_cmd(&temp);


	BCT_activate_block();
}

static void BC_load_uvc_lv3_action(size_t pos)	//Safe mode, STx OFF, Dur -> 10, 4000
{
	CTCP temp;

	BCT_clear_block(pos);

	temp = start_transition(SAFE);
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(2));
	BCT_register_cmd(&temp);

	temp = cmd_stx_set_duty(5, 4000);
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(20));
	BCT_register_cmd(&temp);

	temp = cmd_stx_off();
	CCP_convert_rt_cmd_to_tl_cmd(&temp, OBCT_sec2cycle(21));
	BCT_register_cmd(&temp);

	BCT_activate_block();
}