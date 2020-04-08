// 
// 
// 

#include "UVC_app.h"

#include <stddef.h> // for NULL
#include <string.h> // for memcpy

#include "cmd_analyze_obc.h"
#include "BlockCommandDefinisions.h"
#include "utility.h"

UVCInfo uvci;

static void UVC_init_(void);
static void UVC_execute_(void);

AppInfo UVC_create_app(void)
{
	return create_app_info("uvc", 5, UVC_init_, UVC_execute_);
}

static void UVC_init_(void)
{
	ENABLE_STS ena = DISABLE;
	unsigned char v1, v2, v3;
	uvci.level = UVC_SUCCESS;

	FRAM_Read_uvc_ena(&ena);
	FRAM_Read_uvc_threshold(&v1, &v2, &v3);

	uvci.is_enabled = (unsigned char)ena;
	if (UVC_set(v1, v2, v3) != 0)
	{
		uvci.threshold[0] = UVC_DEFAULT_THRESHOLD_LV1;
		uvci.threshold[1] = UVC_DEFAULT_THRESHOLD_LV2;
		uvci.threshold[2] = UVC_DEFAULT_THRESHOLD_LV3;
	}
}

int	UVC_set(unsigned char v1, unsigned char v2, unsigned char v3)
{
	int i_ret = 0;

	if (v1 == 0 || v2 == 0 || v3 == 0)
		return (-1);

	if (v1 < v2 || v1 < v3)
		return (-1);

	if (v2 < v3)
		return (-1);

	uvci.threshold[0] = v1;
	uvci.threshold[1] = v2;
	uvci.threshold[2] = v3;

	FRAM_Write_uvc_threshold(v1, v2, v3);
	return i_ret;
}

void UVC_enable(unsigned char en)
{
	uvci.is_enabled = en;
	FRAM_Write_uvc_ena((ENABLE_STS)en);
}

static void UVC_execute_(void)
{
	if (uvci.is_enabled)
	{
		if (eps_bat_v < ((float)uvci.threshold[2] / 10))
		{
			uvci.level = UVC_LEVEL3;
			AL_add_anomaly(AL_UVC, UVC_LEVEL3);
		}
		else if (eps_bat_v < ((float)uvci.threshold[1] / 10))
		{
			uvci.level = UVC_LEVEL2;
			AL_add_anomaly(AL_UVC, UVC_LEVEL2);
		}
		else if (eps_bat_v < ((float)uvci.threshold[0] / 10))
		{
			uvci.level = UVC_LEVEL1;
			AL_add_anomaly(AL_UVC, UVC_LEVEL1);
		}
		
	}
}
