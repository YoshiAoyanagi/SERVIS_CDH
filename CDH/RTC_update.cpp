// 
// 
// 

#include "RTC_update.h"

void RTC_update_init_(void);
void RTC_update_(void);


AppInfo RTC_update(void)
{
	return create_app_info("RTC_UD", 10, RTC_update_init_, RTC_update_);
}

void RTC_update_init_(void)
{
	RTC_get_UNIX_TIME();
}

void RTC_update_(void)
{
	static unsigned int c;
	if (c >= (1.0 * OBCT_CYCLES_PER_SEC))
	{
		RTC_get_UNIX_TIME();
		c = 0;
	}
	c++;
}