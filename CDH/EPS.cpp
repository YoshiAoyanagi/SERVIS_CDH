// 
// 
// 

#include "EPS.h"
#include "MTQ_OBC.h"

PWR_STS	pwr_sts;
ack eps_access_sts;
ack pcu_access_sts;
ack pdu_access_sts;

unsigned char pcu_select;
float eps_bat_v;
static cycle_t eps_interval_cycle;

ack EPS_init(void)
{
	pinMode(PORT_PCU_SELECT, OUTPUT);

	pinMode(PORT_PCU_RESET1, OUTPUT);
	pinMode(PORT_PCU_RESET2, OUTPUT);
	pinMode(PORT_PCU_SELECT_BAT1, OUTPUT);
	pinMode(PORT_PCU_SELECT_BAT2, OUTPUT);
	pinMode(PORT_HEATER_PWR, OUTPUT);

	pinMode(PORT_STRX_PWR, OUTPUT);
	digitalWrite(PORT_STRX_PWR, LOW);

	digitalWrite(PORT_PCU_RESET1, LOW);
	digitalWrite(PORT_PCU_RESET2, LOW);

	digitalWrite(PORT_PCU_SELECT_BAT2, LOW);
	digitalWrite(PORT_PCU_SELECT_BAT1, LOW);

	digitalWrite(PORT_HEATER_PWR, LOW);

	EPS_SELECT(EPS_SYS_A);

	if (pdu_access_sts != Success || pcu_access_sts != Success)
	{
		Serial.print("EPS failed: change ch-B");
		delay(100);
		EPS_SELECT(EPS_SYS_B);
	}

	if (pdu_access_sts != Success || pcu_access_sts != Success)
	{
		Serial.print("EPS failed: RESET PCU/PDU board");
		delay(100);
		EPS_RESET();

		delay(100);
		EPS_SELECT(EPS_SYS_A);
	}

	float mv;
	mv = analogRead(PORT_AN_BAT1_MON);
	eps_bat_v = ((mv * 2) * 5.715) / 1000;

	if (pdu_access_sts == Success && pcu_access_sts == Success)
	{
		PCU_Task();
		PDU_Task();
		return Success;
	}
	else
		return Error;
}

void EPS_SELECT(unsigned char sel)
{
	pinMode(PORT_PCU_SELECT, OUTPUT);

	if (sel == EPS_SYS_B)
	{
		digitalWrite(PORT_PCU_SELECT, HIGH);
		pcu_select = EPS_SYS_B;
		Serial.println("Select: EPS_SYS_B");
		if ((pcu_access_sts = PCU_init()) != Success)
		{
			Serial.println("Failed: Initialize PCU");
		}
		delay(100);

		if ((pdu_access_sts = PDU_init()) != Success)
		{
			Serial.println("Failed: Initialize PDU");

		}
	}
	else
	{
		digitalWrite(PORT_PCU_SELECT, LOW);
		pcu_select = EPS_SYS_A;
		Serial.println("Select: EPS_SYS_A");
		if ((pcu_access_sts = PCU_init()) != Success)
		{
			Serial.println("Failed: Initialize PCU");
		}
		delay(100);

		if ((pdu_access_sts = PDU_init()) != Success)
		{
			Serial.println("Failed: Initialize PDU");
		}
	}
}
void EPS_GetStatus(void)
{
	float mv;
	static unsigned int c;

	mv = analogRead(PORT_AN_BAT1_MON);
	eps_bat_v = ((mv * 2) * 5.715) / 1000;

	if (pcu_access_sts == Success) { PCU_Task(); }
	//else { PCU_init(); }

	if (pdu_access_sts == Success) { PDU_Task(); }
	//else { PDU_init(); }

	return;
}

ack EPS_RESET(void)
{

		digitalWrite(PORT_PCU_RESET2, HIGH);
		delay(1000);
		digitalWrite(PORT_PCU_RESET2, LOW);
		
		delay(500);
		digitalWrite(PORT_PCU_RESET1, HIGH);
		delay(1000);
		digitalWrite(PORT_PCU_RESET1, LOW);
	
	return Success;
}

ack		PWR_PDU_SET(unsigned char pwr_onoff, unsigned char on_port, unsigned char off_port)
{
	ack i_ack = Error;

	if (pdu_access_sts == Success)
	{

		if (pwr_onoff == PWR_ON)
		{
			i_ack = PDU_IO_Write(off_port, LOW);
			i_ack = PDU_IO_Write(on_port, HIGH);
		}
		else
		{
			i_ack = PDU_IO_Write(off_port, HIGH);
			i_ack = PDU_IO_Write(on_port, LOW);
		}
		delay(200);
		i_ack = PDU_IO_Write(off_port, LOW);
		i_ack = PDU_IO_Write(on_port, LOW);
	}
	EPS_GetStatus();
	return i_ack;
}

ack		PWR_PDU_UNREG_SET(unsigned char pwr_onoff, unsigned char on_port, unsigned char off_port)
{
	ack i_ack = Error;

	if (pdu_access_sts == Success)
	{

		if (pwr_onoff == PWR_ON)
		{
			i_ack = PDU_IO_Write(off_port, LOW);
			i_ack = PDU_IO_Write(on_port, HIGH);
		}
		else
		{
			i_ack = PDU_IO_Write(off_port, HIGH);
			i_ack = PDU_IO_Write(on_port, LOW);

			delay(200);
			i_ack = PDU_IO_Write(off_port, LOW);
		}
	}
	EPS_GetStatus();
	return i_ack;
}

ack		PWR_UNREG_OBC_SET(unsigned char pwr_onoff)
{
	ack i_ack = Error;

	pinMode(PORT_UNREG_ON, OUTPUT);
	pinMode(PORT_UNREG_OFF, OUTPUT);

	if (pwr_onoff == PWR_ON)
	{
		digitalWrite(PORT_UNREG_ON, HIGH);
		digitalWrite(PORT_UNREG_OFF, LOW);
	}
	else
	{
		digitalWrite(PORT_UNREG_ON, LOW);
		digitalWrite(PORT_UNREG_OFF, HIGH);
	}
	delay(100);
	digitalWrite(PORT_UNREG_ON, LOW);
	digitalWrite(PORT_UNREG_OFF, LOW);

	return i_ack;
}

ack PWR_PORT_SET(unsigned char pwr_onoff, unsigned char pwr_port)
{
	ack i_ack;
	pinMode(pwr_port, OUTPUT);

	if (pwr_onoff == PWR_ON)
	{
		digitalWrite(pwr_port, HIGH);
	}
	else
	{
		digitalWrite(pwr_port, LOW);
	}

	EPS_GetStatus();
	return Success;
}

ack		PWR_SAFE_OFF(void)
{
	PWR_UNREG(PWR_OFF);
	PWR_HEATER(PWR_OFF);
	PWR_RW(PWR_OFF);
	PWR_MSN5V(PWR_OFF);
	PWR_SUBCAM(PWR_OFF);
	PWR_SF(PWR_OFF);
	PWR_RW_ENABLE(PWR_OFF);
	PWR_MSN_UNREG_ENABLE(PWR_OFF);
}

ack		PWR_UNREG(unsigned char pwr_onoff)
{
	ack i_ret;
	//i_ret = PWR_PDU_SET(pwr_onoff, UNREG_ON_PORT, UNREG_OFF_PORT);
	i_ret = PWR_PDU_UNREG_SET(pwr_onoff, UNREG_ON_PORT, UNREG_OFF_PORT);
	//i_ret = PWR_PDU_UNREG_SET(pwr_onoff, UNREG_ON_PORT, UNREG_OFF_PORT);
	//i_ret = PWR_UNREG_OBC_SET(pwr_onoff);

	if (i_ret != Error)
	{
		pwr_sts.msn_unreg = pwr_onoff;
		FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_UNREG, pwr_onoff);
	}
	return i_ret;
}

ack		PWR_RW(unsigned char pwr_onoff)
{
	ack i_ret = PWR_PDU_UNREG_SET(pwr_onoff, RW_ON_PORT, RW_OFF_PORT);
	if (i_ret != Error)
	{
		pwr_sts.rw = pwr_onoff;
		FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_RW, pwr_onoff);
	}
}
ack		PWR_ADCS(unsigned char pwr_onoff)
{
	ack i_ret = PWR_PDU_SET(pwr_onoff, ADCS_ON_PORT, ADCS_OFF_PORT);
	if (i_ret != Error)
	{
		pwr_sts.adcs = pwr_onoff;
		FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_ADCS, pwr_onoff);
	}

	if (pwr_onoff == PWR_ON)
	{
		if (MTQ_IOE_init() == 1) { DEBUG_Serial.println("Success MTQ_IOE initialize"); }
		else { DEBUG_Serial.println("Failed MTQ_IOE"); }
	}
}
ack		PWR_MSN5V(unsigned char pwr_onoff)
{
	ack i_ret = PWR_PDU_SET(pwr_onoff, MSN5V_ON_PORT, MSN5V_OFF_PORT);
	if (i_ret != Error)
	{
		pwr_sts.msn5v = pwr_onoff;
		FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_MSN5V, pwr_onoff);
	}
}
ack		PWR_SUBCAM(unsigned char pwr_onoff)
{
	ack i_ret = PWR_PDU_SET(pwr_onoff, SUBCAM_ON_PORT, SUBCAM_OFF_PORT);
	if (i_ret != Error)
	{
		pwr_sts.subcam = pwr_onoff;
		FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_SUBCAM, pwr_onoff);
	}
}
ack		PWR_SF(unsigned char pwr_onoff)
{
	ack i_ret = PWR_PDU_SET(pwr_onoff, SF_ON_PORT, SF_OFF_PORT);
	if (i_ret != Error)
	{
		pwr_sts.sf = pwr_onoff;
		FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_SF, pwr_onoff);
	}
}

ack		PWR_MSN_UNREG_ENABLE(unsigned char pwr_onoff)
{
	TRISDbits.TRISD2 = 0;
	LATDbits.LATD2 = !pwr_onoff;
	pwr_sts.msn_unreg_en = pwr_onoff;
	FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_MSN_UNREG_EN, pwr_onoff);
	return Success;
}
ack		PWR_RW_ENABLE(unsigned char pwr_onoff)
{
	ack i_ret;

	TRISDbits.TRISD3 = 0;
	LATDbits.LATD3 = !pwr_onoff;
	pwr_sts.rw_en = pwr_onoff;
	FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_RW_EN, pwr_onoff);
	return i_ret;
}
ack		PWR_HEATER(unsigned char pwr_onoff)
{
	ack i_ret;

	TRISDbits.TRISD1 = 0;
	LATDbits.LATD1 = pwr_onoff;	//‘¼‚Æ‚ÍHigh/Low‚ª‹t
	pwr_sts.heater = pwr_onoff;
	FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_HEATER, pwr_onoff);
	return i_ret;
}
ack		PWR_IMU(unsigned char pwr_onoff)
{
	ack i_ret = PWR_PORT_SET(pwr_onoff, PORT_IMU_PWR);
	if (i_ret != Error)
		pwr_sts.imu = pwr_onoff;
	else
		pwr_sts.imu = PWR_ERROR;
	return i_ret;
}

ack		PWR_STRX(unsigned char pwr_onoff)
{
	ack i_ret = PWR_PORT_SET(pwr_onoff, PORT_STRX_PWR);
	if (i_ret != Error)
		pwr_sts.strx = pwr_onoff;
	else
		pwr_sts.strx = PWR_ERROR;
	return i_ret;
}

ack		PWR_AQU_5V(unsigned char pwr_onoff)
{
	PWR_MSN5V(pwr_onoff);

	ack i_ret = PWR_PORT_SET(pwr_onoff, PORT_AQU_5V_PWR);
	if (i_ret != Error)
		pwr_sts.aqu_5v = pwr_onoff;
	else
		pwr_sts.aqu_5v = PWR_ERROR;

	FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_AQU_5V, pwr_onoff);
	return i_ret;
}

ack		PWR_OVCO_5V(unsigned char pwr_onoff)
{
	PWR_MSN5V(pwr_onoff);

	ack i_ret = PWR_PORT_SET(pwr_onoff, PORT_OPT_5V_PWR);
	if (i_ret != Error)
		pwr_sts.ovco = pwr_onoff;
	else
		pwr_sts.ovco = PWR_ERROR;

	FRAM_Write_PWR_STS(FRAM_ADDRESS_PWR_OVCO_5V, pwr_onoff);
	return i_ret;
}


void		PWR_STRX_RESET(void)
{

	digitalWrite(PORT_STRX_PWR, LOW);
	delay(500);
	digitalWrite(PORT_STRX_PWR, HIGH);
	delay(1000);
	digitalWrite(PORT_STRX_PWR, LOW);

	pwr_sts.strx = PWR_ON;

	return;
}
