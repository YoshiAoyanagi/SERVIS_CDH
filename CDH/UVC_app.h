// UVC_app.h

#ifndef _UVC_APP_h
#define _UVC_APP_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "AppInfo.h"
#include "CommonTlmCmdPacket.h"

#define UVC_DEFAULT_THRESHOLD_LV1	65
#define UVC_DEFAULT_THRESHOLD_LV2	63
#define UVC_DEFAULT_THRESHOLD_LV3	62

typedef enum
{
	UVC_SUCCESS,
	UVC_LEVEL1,
	UVC_LEVEL2,
	UVC_LEVEL3,
	UVC_UNKNOWN
} UVC_ACK;

typedef struct
{
	unsigned char is_enabled;
	unsigned char threshold[3];
	unsigned char level;
} UVCInfo;

extern UVCInfo uvci;

AppInfo UVC_create_app(void);
int	UVC_set(unsigned char v1, unsigned char v2, unsigned char v3);
void UVC_enable(unsigned char en);

#endif

