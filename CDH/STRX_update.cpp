//
#include "STRX_update.h"
#include "cmd_item.h"
#include "FRAM_manager.h"

static void STRX_init_(void);
static void STRX_update_(void);
static void STX_tx_init_(void);
static void STX_transmit_(void);

unsigned int stx_transmit_on_duration;
unsigned int stx_transmit_off_duration;
unsigned int stx_burst_duration;
unsigned int stx_transmit_count;
unsigned char stx_transmit_on;

unsigned int stx_on_remaining;
unsigned char strx_otc_enable;
unsigned char strx_otc_threshold;

AppInfo STRX_update(void)
{
	return create_app_info("STRXU", 10, STRX_init_, STRX_update_);
}

static void STRX_init_(void)
{
	unsigned char pwr_level;

	strx_otc_enable = 1;
	strx_otc_threshold = STRX_OTC_DEFAULT_TEMPERATURE;

	PWR_STRX_RESET();

	//İ’è“Ç‚İo‚µ
	FRAM_Read_strx_otc_ena(&strx_otc_enable);
	FRAM_Read_strx_otc_thr(&strx_otc_threshold);
	FRAM_Read_strx_power(&pwr_level);	//add 2019/11/28

	delay(1000);
	Serial.print("STRX_PWR: "); Serial.println(pwr_level);

	STRX_Cmd_BR4KBPS();

	if (pwr_level > 0x05)				//’á‚·‚¬‚é’l‚Ìê‡‚Íİ’è‚µ‚È‚¢(16dBmˆÈã‚ÅÄİ’è)
	{
		STRX_Cmd_PWR(pwr_level);			//add 2019/11/28
	}
	else
	{
		STRX_Cmd_PWR(0x0F);
	}
}

static void STRX_update_(void)
{
	STRX_rec();
	STRX_OTC_Detect();
}

void STRX_IF_Cmd(unsigned char param)
{
	unsigned char data[5];
	data[0] = param;
	data[1] = param;
	data[2] = param;
	data[3] = param;
	data[4] = param;

	int i;
	for (i = 0; i < 5; i++)
	{
		DATA_Serial.write(data[i]);
	}
}

void STRX_IF_Cmd_BR4KBPS(void)
{
	STRX_IF_Cmd(STRX_IF_BR4KBPS);
}
void STRX_IF_Cmd_BR32KBPS(void)
{
	STRX_IF_Cmd(STRX_IF_BR32KBPS);
}
void STRX_IF_Cmd_BR64KBPS(void)
{
	STRX_IF_Cmd(STRX_IF_BR64KBPS);
}
void STRX_IF_Cmd_TX_ON(void)
{
#ifndef UART_DEBUG
	STRX_IF_Cmd(STRX_IF_CMD_TX_ON);
#endif
	//Serial.print("STRX_IF_CMD_TX_ON");
}
void STRX_IF_Cmd_TX_OFF(void)
{
	STRX_IF_Cmd(STRX_IF_CMD_TX_OFF);
	//Serial.print("STRX_IF_CMD_TX_OFF");
}

void STRX_Cmd_BR64KBPS(void)
{
	STRX_set_cmd(strx_sts.pwr_level, STX_BR_64KBPS);
	STRX_IF_Cmd_BR64KBPS();
}

void STRX_Cmd_BR32KBPS(void)
{
	STRX_set_cmd(strx_sts.pwr_level, STX_BR_32KBPS);
	STRX_IF_Cmd_BR32KBPS();
}

void STRX_Cmd_BR4KBPS(void)
{
	STRX_set_cmd(strx_sts.pwr_level, STX_BR_4KBPS);
	STRX_IF_Cmd_BR4KBPS();
}

void STRX_Cmd_PWR(unsigned char pwr_level)
{
	STRX_set_cmd(pwr_level, strx_sts.bitrate);
	FRAM_Write_strx_power(pwr_level);
}

AppInfo STX_transmit(void)
{
	return create_app_info("STXT", 10, STX_tx_init_, STX_transmit_);
}

static void STX_tx_init_(void)
{
	unsigned int tmp_on_dur;
	unsigned int tmp_off_dur;

	//STX_set_ON(); //comment out 2019/11/28
	stx_burst_duration = 0;

	stx_transmit_on_duration = DEFAULT_STX_TRANSMIT_ON_DURATION;
	stx_transmit_off_duration = DEFAULT_STX_TRANSMIT_OFF_DURATION;

	FRAM_Read_stx_on_off_duration(&tmp_on_dur, &tmp_off_dur);

	stx_transmit_on_duration = tmp_on_dur;
	stx_transmit_off_duration = tmp_off_dur;

	if (stx_transmit_on_duration == 0)
	{
		STX_set_OFF();
	}
	else
	{
		STX_set_ON();
	}
}

static void STX_transmit_(void)
{
	if (stx_burst_duration > 0)
	{
		if (stx_transmit_count >= (stx_burst_duration * OBCT_CYCLES_PER_SEC))
		{
			STX_set_OFF();
			stx_burst_duration = 0;
			stx_on_remaining = 0;
		}
		else
			stx_on_remaining = stx_burst_duration - (stx_transmit_count / OBCT_CYCLES_PER_SEC);
	}
	else
	{
		if (stx_transmit_on == 0)
		{
			if (stx_transmit_count >= (stx_transmit_off_duration * OBCT_CYCLES_PER_SEC))
			{
				STX_set_ON();
			}
		}
		else
		{
			if (stx_transmit_count >= (stx_transmit_on_duration * OBCT_CYCLES_PER_SEC))
			{
				STX_set_OFF();
				stx_on_remaining = 0;
			}
			else
				stx_on_remaining = stx_transmit_on_duration - (stx_transmit_count / OBCT_CYCLES_PER_SEC);
		}
	}
	stx_transmit_count++;
}

//setting
void STX_set_ON(void)
{
	stx_transmit_on = 1;
	stx_transmit_count = 0;
	STRX_IF_Cmd_TX_ON();
}
void STX_set_OFF(void)
{
	stx_transmit_on = 0;
	stx_transmit_count = 0;
	STRX_IF_Cmd_TX_OFF();
}


void STX_set_transmit_duration(unsigned int on_duration, unsigned int off_duration)
{
	stx_transmit_on_duration = on_duration;
	stx_transmit_off_duration = off_duration;

	if (on_duration > (30 * 60))//if (on_duration == 0 || on_duration > (30*60))
		return;
	if (off_duration == 0)
		return;

	FRAM_Write_stx_on_off_duration(on_duration, off_duration);

	STX_set_ON();
}

void STX_set_burst_duration(unsigned int duration)
{
	stx_burst_duration = duration;
	STX_set_ON();
}

void STRX_OTC_Detect(void)
{
	static int otc_count;
	CTCP packet;
	unsigned char param = (unsigned char)PWR_OFF;

	unsigned char tmp = (unsigned char)(strx_sts.tempe - 100);
	if (strx_otc_enable && tmp > strx_otc_threshold)
	{
		otc_count++;
		if (otc_count > STRX_OTC_THR_COUNT)
		{
			otc_count = 0;
			packet = CCP_form_rt_cmd(CODE_Cmd_STRX_TX_ON, &param, 1u);
			RT_Cmd_Register(packet);
		}
	}
	else
	{
		otc_count = 0;
	}
}

void STRX_set_OTC_Enable(unsigned char en)
{
	if (en > 1)
		return;

	strx_otc_enable = en;
	FRAM_Write_strx_otc_ena(en);
}

void STRX_set_OTC_Threshold(unsigned char val)
{
	strx_otc_threshold = val;
	FRAM_Write_strx_otc_thr(val);
}